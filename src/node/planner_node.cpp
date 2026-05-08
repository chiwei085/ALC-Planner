#include "alc_planner/planner_node.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace alc_planner
{

namespace
{

Pose6f toPose6f(const geometry_msgs::msg::Pose& pose_msg) {
    Pose6f pose;
    pose.position = Eigen::Vector3f(static_cast<float>(pose_msg.position.x),
                                    static_cast<float>(pose_msg.position.y),
                                    static_cast<float>(pose_msg.position.z));
    pose.orientation =
        Eigen::Quaternionf(static_cast<float>(pose_msg.orientation.w),
                           static_cast<float>(pose_msg.orientation.x),
                           static_cast<float>(pose_msg.orientation.y),
                           static_cast<float>(pose_msg.orientation.z))
            .normalized();
    return pose;
}

std::optional<int> extractWordsRecognized(const rtabmap_msgs::msg::Info& msg) {
    static const std::vector<std::string> kCandidates = {
        "Keypoint/Words", "Keypoint/Words_Recognized",
        "Keypoint/Words recognized", "Keypoint/Dictionary_size/words"};

    for (const auto& key : kCandidates) {
        for (std::size_t i = 0;
             i < msg.stats_keys.size() && i < msg.stats_values.size(); ++i) {
            if (msg.stats_keys[i] == key) {
                return static_cast<int>(std::lround(msg.stats_values[i]));
            }
        }
    }
    return std::nullopt;
}

}  // namespace

ALCPlannerNode::ALCPlannerNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("alc_planner", opts),
      saliency_eval_(params_),
      candidate_builder_(params_),
      reward_evaluator_(params_) {
    sub_map_data_ = create_subscription<rtabmap_msgs::msg::MapData>(
        "/rtabmap/mapData", rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onMapData, this, std::placeholders::_1));
    sub_info_ = create_subscription<rtabmap_msgs::msg::Info>(
        "/rtabmap/info", rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onInfo, this, std::placeholders::_1));
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onMap, this, std::placeholders::_1));
}

void ALCPlannerNode::onMapData(
    const rtabmap_msgs::msg::MapData::SharedPtr msg) {
    if (!msg) {
        return;
    }

    ingestNodes(*msg);
    ingestLinks(*msg);
    saliency_eval_.update(graph_);
    checkLighthouse();
    candidates_ = candidate_builder_.build(graph_);
    for (auto& candidate : candidates_) {
        reward_evaluator_.fillRewardUB(candidate, graph_);
    }
    last_map_data_stamp_ = msg->header.stamp;
    logGraphState();
}

void ALCPlannerNode::onInfo(const rtabmap_msgs::msg::Info::SharedPtr msg) {
    if (!msg) {
        return;
    }

    if (const auto words_recognized = extractWordsRecognized(*msg);
        words_recognized.has_value()) {
        saliency_eval_.observeWordsRecognized(*words_recognized);
    }
    else {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "[ALCPlanner] /rtabmap/info stats_keys did not match any known "
            "words-recognized key; S_L normalization may be incorrect.");
    }

    graph_.robot_node_id = msg->ref_id;
    if (msg->loop_closure_id > 0) {
        RCLCPP_INFO(get_logger(), "[ALCPlanner] loop closure detected: id=%d",
                    msg->loop_closure_id);
    }
}

void ALCPlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    occupancy_map_ = msg;
}

void ALCPlannerNode::ingestNodes(const rtabmap_msgs::msg::MapData& msg) {
    for (const auto& node_msg : msg.nodes) {
        auto& keyframe = graph_.keyframes[node_msg.id];
        keyframe.node_id = node_msg.id;
        keyframe.pose = toPose6f(node_msg.pose);
        keyframe.word_ids = node_msg.word_id_keys;
    }

    const std::size_t pose_count =
        std::min(msg.graph.poses_id.size(), msg.graph.poses.size());
    for (std::size_t i = 0; i < pose_count; ++i) {
        const int node_id = msg.graph.poses_id[i];
        auto& keyframe = graph_.keyframes[node_id];
        keyframe.node_id = node_id;
        keyframe.pose = toPose6f(msg.graph.poses[i]);
    }
}

void ALCPlannerNode::ingestLinks(const rtabmap_msgs::msg::MapData& msg) {
    graph_.adj.clear();
    for (const auto& [node_id, keyframe] : graph_.keyframes) {
        (void)keyframe;
        graph_.adj[node_id];
    }

    for (const auto& link : msg.graph.links) {
        const auto it_from = graph_.keyframes.find(link.from_id);
        const auto it_to = graph_.keyframes.find(link.to_id);
        if (it_from == graph_.keyframes.end() ||
            it_to == graph_.keyframes.end()) {
            continue;
        }

        const float dist =
            (it_from->second.pose.position - it_to->second.pose.position)
                .norm();
        graph_.adj[link.from_id].push_back({link.to_id, dist});
        graph_.adj[link.to_id].push_back({link.from_id, dist});
    }

    RCLCPP_DEBUG(get_logger(),
                 "[ALCPlanner] ingestLinks: rebuilt %zu directed edges",
                 msg.graph.links.size() * 2U);
}

void ALCPlannerNode::checkLighthouse() {
    if (graph_.robot_node_id < 0) {
        return;
    }

    const auto it = graph_.keyframes.find(graph_.robot_node_id);
    if (it == graph_.keyframes.end()) {
        return;
    }

    Keyframe& current = it->second;
    if (current.saliency_local <= params_.cs_lighthouse) {
        return;
    }

    const float dist_from_last =
        has_lighthouse_
            ? (current.pose.position - last_lighthouse_pose_.position).norm()
            : std::numeric_limits<float>::max();
    if (dist_from_last <= params_.d_min_lighthouse) {
        return;
    }

    current.is_lighthouse = true;
    last_lighthouse_pose_ = current.pose;
    has_lighthouse_ = true;
    RCLCPP_INFO(get_logger(), "[ALCPlanner] lighthouse at node %d (S_L=%.3f)",
                current.node_id, current.saliency_local);
}

void ALCPlannerNode::logGraphState() const {
    std::size_t edge_count = 0;
    for (const auto& [node_id, edges] : graph_.adj) {
        (void)node_id;
        edge_count += edges.size();
    }

    RCLCPP_DEBUG(get_logger(), "[ALCPlanner] graph: %zu nodes, %zu edges",
                 graph_.keyframes.size(), edge_count / 2U);

    std::vector<const Keyframe*> ordered_keyframes;
    ordered_keyframes.reserve(graph_.keyframes.size());
    for (const auto& [node_id, keyframe] : graph_.keyframes) {
        (void)node_id;
        ordered_keyframes.push_back(&keyframe);
    }
    std::sort(ordered_keyframes.begin(), ordered_keyframes.end(),
              [](const Keyframe* lhs, const Keyframe* rhs) {
                  return lhs->node_id < rhs->node_id;
              });

    for (const Keyframe* keyframe_ptr : ordered_keyframes) {
        const Keyframe& keyframe = *keyframe_ptr;
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] node %d: S_L=%.3f S_G=%.3f "
                     "plc_intrinsic=%.3f words=%zu",
                     keyframe.node_id, keyframe.saliency_local,
                     keyframe.saliency_global, keyframe.plc_intrinsic,
                     keyframe.word_ids.size());
    }

    RCLCPP_DEBUG(get_logger(), "[ALCPlanner] candidates: %zu",
                 candidates_.size());
    for (const auto& candidate : candidates_) {
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] cand tau_id=%d euclid=%.2f graph_dist=%.2f "
                     "reward_ub=%.4f lighthouse=%d cluster_size=%zu",
                     candidate.tau_id, candidate.euclidean_dist,
                     candidate.graph_dist, candidate.reward_ub,
                     static_cast<int>(candidate.is_lighthouse),
                     candidate.keyframe_ids.size());
    }

    if (graph_.robot_node_id < 0) {
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] robot_node_id unavailable, skipping graph "
                     "distance logging");
        return;
    }

    const auto dist_map =
        UncertaintyMetrics::dijkstraAll(graph_, graph_.robot_node_id);
    if (dist_map.empty()) {
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] robot node %d not found in graph, skipping "
                     "graph distance logging",
                     graph_.robot_node_id);
        return;
    }

    std::vector<std::pair<int, float>> ordered_distances(dist_map.begin(),
                                                         dist_map.end());
    std::sort(
        ordered_distances.begin(), ordered_distances.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    for (const auto& [node_id, dist] : ordered_distances) {
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] graph_dist robot=%d -> node=%d : %.3f",
                     graph_.robot_node_id, node_id, dist);
    }
}

}  // namespace alc_planner
