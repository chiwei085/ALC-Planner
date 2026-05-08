#include "alc_planner/candidate_builder.hpp"

#include <Eigen/Core>

#include "alc_planner/uncertainty_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace alc_planner
{

namespace
{

constexpr int kUnvisited = -2;
constexpr int kNoise = -1;

float positionDistance(const GraphState& graph, const int lhs_id,
                       const int rhs_id) {
    const auto lhs_it = graph.keyframes.find(lhs_id);
    const auto rhs_it = graph.keyframes.find(rhs_id);
    if (lhs_it == graph.keyframes.end() || rhs_it == graph.keyframes.end()) {
        return std::numeric_limits<float>::infinity();
    }

    return (lhs_it->second.pose.position - rhs_it->second.pose.position).norm();
}

std::vector<int> regionQuery(const GraphState& graph,
                             const std::vector<int>& ids, const int query_id,
                             const float eps_dbscan) {
    std::vector<int> neighbors;
    neighbors.reserve(ids.size());
    for (const int other_id : ids) {
        if (positionDistance(graph, query_id, other_id) <= eps_dbscan) {
            neighbors.push_back(other_id);
        }
    }
    return neighbors;
}

}  // namespace

CandidateBuilder::CandidateBuilder(Params params) : params_(params) {}

std::vector<ALCCandidate> CandidateBuilder::build(
    const GraphState& graph) const {
    const auto robot_it = graph.keyframes.find(graph.robot_node_id);
    if (robot_it == graph.keyframes.end()) {
        return {};
    }

    const auto dist_map =
        UncertaintyMetrics::dijkstraAll(graph, graph.robot_node_id);

    const std::vector<int> filtered = filterKeyframes(graph, dist_map);
    if (filtered.empty()) {
        return {};
    }

    const std::vector<std::vector<int>> clusters = dbscan(graph, filtered);
    const Pose6f& robot_pose = robot_it->second.pose;

    std::vector<ALCCandidate> candidates;
    candidates.reserve(clusters.size());
    for (const auto& cluster : clusters) {
        const int rep_id = selectRepresentative(graph, cluster, robot_pose);
        if (rep_id < 0) {
            continue;
        }

        const auto rep_it = graph.keyframes.find(rep_id);
        if (rep_it == graph.keyframes.end()) {
            continue;
        }

        ALCCandidate candidate;
        candidate.tau_id = rep_id;
        candidate.rep_pose = rep_it->second.pose;
        candidate.keyframe_ids = cluster;
        candidate.euclidean_dist =
            (robot_pose.position - candidate.rep_pose.position).norm();
        const auto dist_it = dist_map.find(rep_id);
        candidate.graph_dist = dist_it != dist_map.end()
                                   ? dist_it->second
                                   : std::numeric_limits<float>::infinity();
        candidate.is_lighthouse = rep_it->second.is_lighthouse;
        candidates.push_back(std::move(candidate));
    }

    return candidates;
}

std::vector<int> CandidateBuilder::filterKeyframes(
    const GraphState& graph,
    const std::unordered_map<int, float>& dist_map) const {
    const auto robot_it = graph.keyframes.find(graph.robot_node_id);
    if (robot_it == graph.keyframes.end()) {
        return {};
    }

    const Pose6f& robot_pose = robot_it->second.pose;
    std::vector<int> result;
    result.reserve(graph.keyframes.size());

    for (const auto& [node_id, keyframe] : graph.keyframes) {
        if (node_id == graph.robot_node_id) {
            continue;
        }

        const float euclidean_dist =
            (keyframe.pose.position - robot_pose.position).norm();
        if (euclidean_dist > params_.cE) {
            continue;
        }

        if (keyframe.saliency_local < params_.cs) {
            continue;
        }

        const auto dist_it = dist_map.find(node_id);
        const float graph_dist = dist_it != dist_map.end()
                                     ? dist_it->second
                                     : std::numeric_limits<float>::infinity();
        if (graph_dist < params_.cG) {
            continue;
        }

        result.push_back(node_id);
    }

    return result;
}

std::vector<std::vector<int>> CandidateBuilder::dbscan(
    const GraphState& graph, const std::vector<int>& ids) const {
    std::unordered_map<int, int> labels;
    labels.reserve(ids.size());
    for (const int id : ids) {
        labels.emplace(id, kUnvisited);
    }

    std::vector<std::vector<int>> clusters;
    for (const int point_id : ids) {
        if (labels[point_id] != kUnvisited) {
            continue;
        }

        const std::vector<int> neighbors =
            regionQuery(graph, ids, point_id, params_.eps_dbscan);
        if (static_cast<int>(neighbors.size()) < params_.min_pts) {
            labels[point_id] = kNoise;
            continue;
        }

        const int cluster_label = static_cast<int>(clusters.size());
        std::vector<int> cluster;
        cluster.push_back(point_id);
        labels[point_id] = cluster_label;

        std::vector<int> seed_set;
        seed_set.reserve(neighbors.size());
        for (const int neighbor_id : neighbors) {
            if (neighbor_id != point_id) {
                seed_set.push_back(neighbor_id);
            }
        }

        for (std::size_t seed_index = 0; seed_index < seed_set.size();
             ++seed_index) {
            const int neighbor_id = seed_set[seed_index];
            const int prev_label = labels[neighbor_id];
            if (prev_label == kNoise) {
                labels[neighbor_id] = cluster_label;
                cluster.push_back(neighbor_id);
                continue;
            }
            if (prev_label != kUnvisited) {
                continue;
            }

            labels[neighbor_id] = cluster_label;
            cluster.push_back(neighbor_id);

            const std::vector<int> neighbor_neighbors =
                regionQuery(graph, ids, neighbor_id, params_.eps_dbscan);
            if (static_cast<int>(neighbor_neighbors.size()) < params_.min_pts) {
                continue;
            }

            for (const int expanded_id : neighbor_neighbors) {
                if (labels[expanded_id] == kUnvisited ||
                    labels[expanded_id] == kNoise) {
                    seed_set.push_back(expanded_id);
                }
            }
        }

        clusters.push_back(std::move(cluster));
    }

    return clusters;
}

int CandidateBuilder::selectRepresentative(const GraphState& graph,
                                           const std::vector<int>& cluster_ids,
                                           const Pose6f& robot_pose) const {
    float best_score = -1.0f;
    int best_id = -1;

    for (const int node_id : cluster_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        const Keyframe& keyframe = it->second;
        Eigen::Vector3f approach = keyframe.pose.position - robot_pose.position;
        if (approach.norm() < 1e-6f) {
            approach = keyframe.pose.forward();
        }
        else {
            approach.normalize();
        }

        if (approach.dot(keyframe.pose.forward()) < 0.0f) {
            continue;
        }

        if (keyframe.plc_intrinsic > best_score) {
            best_score = keyframe.plc_intrinsic;
            best_id = node_id;
        }
    }

    if (best_id != -1) {
        return best_id;
    }

    best_score = -1.0f;
    for (const int node_id : cluster_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        if (it->second.plc_intrinsic > best_score) {
            best_score = it->second.plc_intrinsic;
            best_id = node_id;
        }
    }

    return best_id;
}

}  // namespace alc_planner
