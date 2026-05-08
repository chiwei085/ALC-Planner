#include "alc_planner/candidate_builder.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "alc_planner/uncertainty_metrics.hpp"

namespace alc_planner
{

namespace
{

constexpr int kUnvisited = -2;
constexpr int kNoise = -1;

struct PointEntry
{
    int ix = -1;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
};

std::vector<int> regionQuery(const std::vector<PointEntry>& points,
                             const std::size_t query_index,
                             const float eps_dbscan) {
    std::vector<int> neighbors;
    neighbors.reserve(points.size());
    const Eigen::Vector3f& query_pos = points[query_index].pos;
    for (const auto& point : points) {
        if ((point.pos - query_pos).norm() <= eps_dbscan) {
            neighbors.push_back(point.ix);
        }
    }
    return neighbors;
}

float distAt(const std::vector<float>& dist_map, const int ix) {
    if (ix < 0 || ix >= static_cast<int>(dist_map.size())) {
        return std::numeric_limits<float>::infinity();
    }
    return dist_map[static_cast<std::size_t>(ix)];
}

}  // namespace

CandidateBuilder::CandidateBuilder(Params params) : params_(params) {}

std::vector<ALCCandidate> CandidateBuilder::build(
    const GraphState& graph, const SaliencyState& saliency_state) {
    if (graph.robot_ix < 0 ||
        graph.robot_ix >= static_cast<int>(graph.keyframes.size())) {
        return {};
    }

    const DijkstraKey cache_key{graph.version, graph.robot_ix};
    bool cache_matches = false;
    if (dijkstra_cache_.has_value()) {
        cache_matches = (dijkstra_cache_->key == cache_key);
    }
    if (!cache_matches) {
        dijkstra_cache_ = DijkstraCache{cache_key, computeDijkstra(graph)};
    }
    const DijkstraResult& dijkstra_result = dijkstra_cache_->result;

    const std::vector<int> filtered =
        filterKeyframes(graph, saliency_state, dijkstra_result.by_dist);
    if (filtered.empty()) {
        return {};
    }

    const std::vector<std::vector<int>> clusters = dbscan(graph, filtered);
    const Pose6f& robot_pose =
        graph.keyframes[static_cast<std::size_t>(graph.robot_ix)].pose;

    std::vector<ALCCandidate> candidates;
    candidates.reserve(clusters.size());
    for (const auto& cluster_ixs : clusters) {
        const int rep_ix = selectRepresentative(graph, saliency_state,
                                                cluster_ixs, robot_pose);
        if (rep_ix < 0 || rep_ix >= static_cast<int>(graph.keyframes.size())) {
            continue;
        }

        const Keyframe& representative =
            graph.keyframes[static_cast<std::size_t>(rep_ix)];

        ALCCandidate candidate;
        candidate.tau_ix = rep_ix;
        candidate.rep_pose = representative.pose;
        candidate.keyframe_ixs = cluster_ixs;
        candidate.euclidean_dist =
            (robot_pose.position - candidate.rep_pose.position).norm();
        candidate.graph_dist = distAt(dijkstra_result.by_dist, rep_ix);
        if (dijkstra_result.by_variance.has_value()) {
            candidate.graph_dist_var =
                distAt(*dijkstra_result.by_variance, rep_ix);
        }
        candidate.is_lighthouse =
            rep_ix < static_cast<int>(saliency_state.keyframes.size())
                ? saliency_state.keyframes[static_cast<std::size_t>(rep_ix)]
                      .is_lighthouse
                : false;
        candidates.push_back(std::move(candidate));
    }

    return candidates;
}

std::vector<int> CandidateBuilder::filterKeyframes(
    const GraphState& graph, const SaliencyState& saliency_state,
    const std::vector<float>& dist_map) const {
    if (graph.robot_ix < 0 ||
        graph.robot_ix >= static_cast<int>(graph.keyframes.size())) {
        return {};
    }

    const Pose6f& robot_pose =
        graph.keyframes[static_cast<std::size_t>(graph.robot_ix)].pose;
    std::vector<int> result;
    result.reserve(graph.keyframes.size());

    for (int ix = 0; ix < static_cast<int>(graph.keyframes.size()); ++ix) {
        if (ix == graph.robot_ix) {
            continue;
        }

        const Keyframe& keyframe =
            graph.keyframes[static_cast<std::size_t>(ix)];
        const float euclidean_dist =
            (keyframe.pose.position - robot_pose.position).norm();
        if (euclidean_dist > params_.cE) {
            continue;
        }
        if (ix >= static_cast<int>(saliency_state.keyframes.size()) ||
            saliency_state.keyframes[static_cast<std::size_t>(ix)]
                    .saliency_local < params_.cs) {
            continue;
        }

        const float graph_dist = distAt(dist_map, ix);
        if (graph_dist < params_.cG) {
            continue;
        }

        result.push_back(ix);
    }

    return result;
}

std::vector<std::vector<int>> CandidateBuilder::dbscan(
    const GraphState& graph, const std::vector<int>& ixs) const {
    std::vector<PointEntry> points;
    points.reserve(ixs.size());
    std::vector<int> point_lookup(graph.keyframes.size(), -1);
    for (const int ix : ixs) {
        if (ix < 0 || ix >= static_cast<int>(graph.keyframes.size())) {
            continue;
        }
        point_lookup[static_cast<std::size_t>(ix)] =
            static_cast<int>(points.size());
        points.push_back(
            {ix, graph.keyframes[static_cast<std::size_t>(ix)].pose.position});
    }

    std::vector<int> labels(points.size(), kUnvisited);
    std::vector<std::vector<int>> clusters;
    for (std::size_t point_index = 0; point_index < points.size();
         ++point_index) {
        const int point_ix = points[point_index].ix;
        if (labels[point_index] != kUnvisited) {
            continue;
        }

        const std::vector<int> neighbors =
            regionQuery(points, point_index, params_.eps_dbscan);
        if (static_cast<int>(neighbors.size()) < params_.min_pts) {
            labels[point_index] = kNoise;
            continue;
        }

        const int cluster_label = static_cast<int>(clusters.size());
        std::vector<int> cluster;
        cluster.push_back(point_ix);
        labels[point_index] = cluster_label;

        std::vector<int> seed_set;
        seed_set.reserve(neighbors.size());
        for (const int neighbor_ix : neighbors) {
            if (neighbor_ix != point_ix) {
                seed_set.push_back(neighbor_ix);
            }
        }

        for (std::size_t seed_index = 0; seed_index < seed_set.size();
             ++seed_index) {
            const int neighbor_ix = seed_set[seed_index];
            const int neighbor_index_lookup =
                point_lookup[static_cast<std::size_t>(neighbor_ix)];
            if (neighbor_index_lookup < 0) {
                continue;
            }

            int& neighbor_label =
                labels[static_cast<std::size_t>(neighbor_index_lookup)];
            const int prev_label = neighbor_label;
            if (prev_label == kNoise) {
                neighbor_label = cluster_label;
                cluster.push_back(neighbor_ix);
                continue;
            }
            if (prev_label != kUnvisited) {
                continue;
            }

            neighbor_label = cluster_label;
            cluster.push_back(neighbor_ix);

            const std::vector<int> neighbor_neighbors = regionQuery(
                points, static_cast<std::size_t>(neighbor_index_lookup),
                params_.eps_dbscan);
            if (static_cast<int>(neighbor_neighbors.size()) < params_.min_pts) {
                continue;
            }

            for (const int expanded_ix : neighbor_neighbors) {
                const int expanded_index_lookup =
                    point_lookup[static_cast<std::size_t>(expanded_ix)];
                if (expanded_index_lookup < 0) {
                    continue;
                }

                const int expanded_label =
                    labels[static_cast<std::size_t>(expanded_index_lookup)];
                if (expanded_label == kUnvisited || expanded_label == kNoise) {
                    seed_set.push_back(expanded_ix);
                }
            }
        }

        clusters.push_back(std::move(cluster));
    }

    return clusters;
}

int CandidateBuilder::selectRepresentative(const GraphState& graph,
                                           const SaliencyState& saliency_state,
                                           const std::vector<int>& cluster_ixs,
                                           const Pose6f& robot_pose) const {
    float best_score = -1.0f;
    int best_ix = -1;

    for (const int ix : cluster_ixs) {
        if (ix < 0 || ix >= static_cast<int>(graph.keyframes.size())) {
            continue;
        }

        const Keyframe& keyframe =
            graph.keyframes[static_cast<std::size_t>(ix)];
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

        const float plc_intrinsic =
            ix < static_cast<int>(saliency_state.keyframes.size())
                ? saliency_state.keyframes[static_cast<std::size_t>(ix)]
                      .plc_intrinsic
                : 0.0f;
        if (plc_intrinsic > best_score) {
            best_score = plc_intrinsic;
            best_ix = ix;
        }
    }

    if (best_ix != -1) {
        return best_ix;
    }

    best_score = -1.0f;
    for (const int ix : cluster_ixs) {
        if (ix < 0 || ix >= static_cast<int>(graph.keyframes.size())) {
            continue;
        }

        const float score =
            ix < static_cast<int>(saliency_state.keyframes.size())
                ? saliency_state.keyframes[static_cast<std::size_t>(ix)]
                      .plc_intrinsic
                : 0.0f;
        if (score > best_score) {
            best_score = score;
            best_ix = ix;
        }
    }

    return best_ix;
}

CandidateBuilder::DijkstraResult CandidateBuilder::computeDijkstra(
    const GraphState& graph) const {
    DijkstraResult result;
    result.by_dist = UncertaintyMetrics::dijkstraAll(graph, graph.robot_ix);
    if (params_.use_variance_uncertainty) {
        result.by_variance =
            UncertaintyMetrics::dijkstraVarianceAll(graph, graph.robot_ix);
    }
    return result;
}

}  // namespace alc_planner
