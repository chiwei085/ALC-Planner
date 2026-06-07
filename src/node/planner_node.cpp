#include "alc_planner/planner_node.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcutils/logging.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "alc_planner/map_utils.hpp"

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

constexpr float kDefaultVariance = 1.0f;
constexpr float kMinApproachHeadingDistance = 1.0e-3f;
constexpr double kDefaultNavigationTimeoutSec = 120.0;
constexpr bool kDefaultUseApproachHeading = true;

float scalarVarianceFromInformation(const std::array<double, 36>& information) {
    constexpr int kDiagonalIndices[6] = {0, 7, 14, 21, 28, 35};

    double reciprocal_sum = 0.0;
    int count = 0;
    for (const int index : kDiagonalIndices) {
        const double value = information[static_cast<std::size_t>(index)];
        if (!std::isfinite(value) || value <= 0.0) {
            continue;
        }

        reciprocal_sum += 1.0 / value;
        ++count;
    }

    if (count <= 0) {
        return kDefaultVariance;
    }
    return static_cast<float>(reciprocal_sum / static_cast<double>(count));
}

}  // namespace

ALCPlannerNode::ALCPlannerNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("alc_planner", opts),
      params_(declarePlannerParams()),
      saliency_eval_(params_),
      candidate_builder_(params_),
      reward_evaluator_(params_),
      bnb_selector_(params_),
      slam_graph_planner_(params_) {
    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "/spin");

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

Params ALCPlannerNode::declarePlannerParams() {
    Params params;

    const auto declare_float = [this](const std::string& name,
                                      const float default_value) {
        return static_cast<float>(declare_parameter<double>(
            name, static_cast<double>(default_value)));
    };
    const auto declare_int = [this](const std::string& name,
                                    const int default_value) {
        return declare_parameter<int>(name, default_value);
    };

    params.cv_L = declare_float("cv_L", params.cv_L);
    params.cv_G = declare_float("cv_G", params.cv_G);
    params.cs_lighthouse = declare_float("cs_lighthouse", params.cs_lighthouse);
    params.d_min_lighthouse =
        declare_float("d_min_lighthouse", params.d_min_lighthouse);
    params.cl = declare_float("cl", params.cl);
    params.ct = declare_float("ct", params.ct);
    params.cE = declare_float("cE", params.cE);
    params.cG = declare_float("cG", params.cG);
    params.cs = declare_float("cs", params.cs);
    params.eps_dbscan = declare_float("eps_dbscan", params.eps_dbscan);
    params.min_pts = declare_int("min_pts", params.min_pts);
    params.theta_max = declare_float("theta_max", params.theta_max);
    params.lambda_decay = declare_float("lambda_decay", params.lambda_decay);
    params.alpha_cov = declare_float("alpha_cov", params.alpha_cov);
    params.tau_min_revisit =
        declare_int("tau_min_revisit", params.tau_min_revisit);
    params.plc_min_revisit =
        declare_float("plc_min_revisit", params.plc_min_revisit);
    params.map_dist_min_revisit =
        declare_float("map_dist_min_revisit", params.map_dist_min_revisit);
    params.use_variance_uncertainty = declare_parameter<bool>(
        "use_variance_uncertainty", params.use_variance_uncertainty);

    navigation_timeout_sec_ = declare_parameter<double>(
        "navigation_timeout_sec", kDefaultNavigationTimeoutSec);
    use_approach_heading_ = declare_parameter<bool>("use_approach_heading",
                                                    kDefaultUseApproachHeading);

    return params;
}

void ALCPlannerNode::onMapData(
    const rtabmap_msgs::msg::MapData::SharedPtr msg) {
    if (!msg) {
        return;
    }

    ingestNodes(*msg);
    ingestLinks(*msg);
    saliency_eval_.update(graph_, saliency_state_);
    checkLighthouse();
    candidates_ = candidate_builder_.build(graph_, saliency_state_);
    for (auto& candidate : candidates_) {
        reward_evaluator_.fillRewardUB(candidate, saliency_state_);
    }
    best_candidate_.reset();
    if (occupancy_map_ && graph_.robot_ix >= 0) {
        best_candidate_ = bnb_selector_.select(
            candidates_, graph_, saliency_state_, *occupancy_map_);
        if (best_candidate_.has_value()) {
            const int tau_node_id = nodeIdFromIx(best_candidate_->tau_ix);
            RCLCPP_INFO(
                get_logger(),
                "[ALCPlanner] BNB best: tau_id=%d reward=%.4f map_dist=%.2f",
                tau_node_id, best_candidate_->reward,
                best_candidate_->map_dist);
        }
    }

    const double elapsed = last_alc_time_.nanoseconds() == 0
                               ? std::numeric_limits<double>::infinity()
                               : (now() - last_alc_time_).seconds();
    const float coverage_ratio = computeCoverageRatio();
    const auto to_navigate = slam_graph_planner_.onEvaluationComplete(
        best_candidate_, elapsed, coverage_ratio, graph_.robot_ix);
    if (to_navigate.has_value()) {
        sendNavGoal(*to_navigate);
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

    const auto robot_it = graph_.node_to_ix.find(msg->ref_id);
    graph_.robot_ix =
        robot_it != graph_.node_to_ix.end() ? robot_it->second : -1;
    if (msg->loop_closure_id > 0) {
        RCLCPP_INFO(get_logger(), "[ALCPlanner] loop closure detected: id=%d",
                    msg->loop_closure_id);
    }

    if (alc_rotation_attempt_active_ && msg->loop_closure_id > 0 &&
        slam_graph_planner_.state() == PlannerState::ROTATING) {
        alc_rotation_observed_loop_closure_ = true;
    }
}

void ALCPlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    occupancy_map_ = msg;
    cached_coverage_ratio_ = estimateCoverageRatio(occupancy_map_.get());
}

void ALCPlannerNode::ingestNodes(const rtabmap_msgs::msg::MapData& msg) {
    const auto ensureIx = [this](const int node_id) {
        const auto it = graph_.node_to_ix.find(node_id);
        if (it != graph_.node_to_ix.end()) {
            return it->second;
        }

        const int ix = static_cast<int>(graph_.keyframes.size());
        graph_.node_to_ix[node_id] = ix;
        graph_.ix_to_node.push_back(node_id);
        graph_.keyframes.push_back({});
        graph_.adj.push_back({});
        return ix;
    };

    for (const auto& node_msg : msg.nodes) {
        const int ix = ensureIx(node_msg.id);
        Keyframe& keyframe = graph_.keyframes[static_cast<std::size_t>(ix)];
        keyframe.node_id = node_msg.id;
        keyframe.pose = toPose6f(node_msg.pose);
        keyframe.word_ids = node_msg.word_id_keys;
    }

    const std::size_t pose_count =
        std::min(msg.graph.poses_id.size(), msg.graph.poses.size());
    for (std::size_t i = 0; i < pose_count; ++i) {
        const int node_id = msg.graph.poses_id[i];
        const int ix = ensureIx(node_id);
        Keyframe& keyframe = graph_.keyframes[static_cast<std::size_t>(ix)];
        keyframe.node_id = node_id;
        keyframe.pose = toPose6f(msg.graph.poses[i]);
    }
}

void ALCPlannerNode::ingestLinks(const rtabmap_msgs::msg::MapData& msg) {
    for (auto& edges : graph_.adj) {
        edges.clear();
    }

    for (const auto& link : msg.graph.links) {
        const auto from_it = graph_.node_to_ix.find(link.from_id);
        const auto to_it = graph_.node_to_ix.find(link.to_id);
        if (from_it == graph_.node_to_ix.end() ||
            to_it == graph_.node_to_ix.end()) {
            continue;
        }

        const int from_ix = from_it->second;
        const int to_ix = to_it->second;
        const float dist =
            (graph_.keyframes[static_cast<std::size_t>(from_ix)].pose.position -
             graph_.keyframes[static_cast<std::size_t>(to_ix)].pose.position)
                .norm();
        const float variance = scalarVarianceFromInformation(link.information);
        graph_.adj[static_cast<std::size_t>(from_ix)].push_back(
            {to_ix, dist, variance});
        graph_.adj[static_cast<std::size_t>(to_ix)].push_back(
            {from_ix, dist, variance});
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] link %d->%d: dist=%.3f variance=%.6f",
                     link.from_id, link.to_id, dist, variance);
    }
    ++graph_.version;

    RCLCPP_DEBUG(get_logger(),
                 "[ALCPlanner] ingestLinks: rebuilt %zu directed edges",
                 msg.graph.links.size() * 2U);
}

void ALCPlannerNode::checkLighthouse() {
    if (graph_.robot_ix < 0 ||
        graph_.robot_ix >= static_cast<int>(graph_.keyframes.size())) {
        return;
    }
    if (graph_.robot_ix >= static_cast<int>(saliency_state_.keyframes.size())) {
        return;
    }

    Keyframe& current =
        graph_.keyframes[static_cast<std::size_t>(graph_.robot_ix)];
    KeyframeSaliency& current_saliency =
        saliency_state_.keyframes[static_cast<std::size_t>(graph_.robot_ix)];
    if (current_saliency.saliency_local <= params_.cs_lighthouse) {
        return;
    }

    const float dist_from_last =
        has_lighthouse_
            ? (current.pose.position - last_lighthouse_pose_.position).norm()
            : std::numeric_limits<float>::max();
    if (dist_from_last <= params_.d_min_lighthouse) {
        return;
    }

    current_saliency.is_lighthouse = true;
    last_lighthouse_pose_ = current.pose;
    has_lighthouse_ = true;
    RCLCPP_INFO(get_logger(), "[ALCPlanner] lighthouse at node %d (S_L=%.3f)",
                current.node_id, current_saliency.saliency_local);
}

void ALCPlannerNode::sendNavGoal(const ALCCandidate& target) {
    if (!nav_client_) {
        (void)slam_graph_planner_.onNavigationResult(false);
        best_candidate_.reset();
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] navigate_to_pose action client unavailable");
        return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
        (void)slam_graph_planner_.onNavigationResult(false);
        best_candidate_.reset();
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] navigate_to_pose action server unavailable");
        return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = target.rep_pose.position.x();
    goal.pose.pose.position.y = target.rep_pose.position.y();
    goal.pose.pose.position.z = target.rep_pose.position.z();
    Eigen::Quaternionf goal_orientation = target.rep_pose.orientation;
    if (use_approach_heading_ && graph_.robot_ix >= 0 &&
        graph_.robot_ix < static_cast<int>(graph_.keyframes.size())) {
        const Pose6f& robot_pose =
            graph_.keyframes[static_cast<std::size_t>(graph_.robot_ix)].pose;
        const Eigen::Vector3f delta =
            target.rep_pose.position - robot_pose.position;
        if (delta.head<2>().norm() > kMinApproachHeadingDistance) {
            const float yaw = std::atan2(delta.y(), delta.x());
            goal_orientation = Eigen::Quaternionf(
                Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
        }
    }
    goal_orientation.normalize();
    goal.pose.pose.orientation.x = goal_orientation.x();
    goal.pose.pose.orientation.y = goal_orientation.y();
    goal.pose.pose.orientation.z = goal_orientation.z();
    goal.pose.pose.orientation.w = goal_orientation.w();

    RCLCPP_INFO(get_logger(),
                "[ALCPlanner] sending nav goal: tau_id=%d x=%.2f y=%.2f",
                nodeIdFromIx(target.tau_ix), target.rep_pose.position.x(),
                target.rep_pose.position.y());

    const std::weak_ptr<ALCPlannerNode> weak_self =
        std::static_pointer_cast<ALCPlannerNode>(shared_from_this());
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [weak_self](const auto& handle) {
        const auto self = weak_self.lock();
        if (!self) {
            return;
        }
        if (handle) {
            self->startNavigationTimeout();
            return;
        }

        self->cancelNavigationTimeout();
        (void)self->slam_graph_planner_.onNavigationResult(false);
        self->best_candidate_.reset();
        RCLCPP_WARN(self->get_logger(),
                    "[ALCPlanner] navigation goal rejected");
    };
    options.result_callback = [weak_self](const auto& result) {
        const auto self = weak_self.lock();
        if (!self) {
            return;
        }
        self->cancelNavigationTimeout();
        const bool success =
            result.code == rclcpp_action::ResultCode::SUCCEEDED;
        const bool should_spin =
            self->slam_graph_planner_.onNavigationResult(success);
        if (should_spin) {
            self->sendSpinGoal();
            return;
        }

        if (!success) {
            self->best_candidate_.reset();
            RCLCPP_WARN(self->get_logger(),
                        "[ALCPlanner] navigation to ALC target failed");
        }
    };

    nav_client_->async_send_goal(goal, options);
}

void ALCPlannerNode::startNavigationTimeout() {
    cancelNavigationTimeout();
    if (navigation_timeout_sec_ <= 0.0) {
        return;
    }

    const std::weak_ptr<ALCPlannerNode> weak_self =
        std::static_pointer_cast<ALCPlannerNode>(shared_from_this());
    nav_timeout_timer_ = create_wall_timer(
        std::chrono::duration<double>(navigation_timeout_sec_), [weak_self]() {
            const auto self = weak_self.lock();
            if (!self) {
                return;
            }
            self->cancelNavigationTimeout();
            if (self->slam_graph_planner_.state() !=
                PlannerState::NAVIGATING_TO_ALC) {
                return;
            }

            (void)self->slam_graph_planner_.onNavigationResult(false);
            self->best_candidate_.reset();
            RCLCPP_WARN(self->get_logger(),
                        "[ALCPlanner] navigation to ALC target timed out "
                        "after %.1f seconds",
                        self->navigation_timeout_sec_);
        });
}

void ALCPlannerNode::cancelNavigationTimeout() {
    if (nav_timeout_timer_) {
        nav_timeout_timer_->cancel();
        nav_timeout_timer_.reset();
    }
}

void ALCPlannerNode::sendSpinGoal() {
    if (!spin_client_) {
        slam_graph_planner_.onRotationComplete();
        last_alc_time_ = now();
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] spin action client unavailable");
        return;
    }
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(0))) {
        slam_graph_planner_.onRotationComplete();
        last_alc_time_ = now();
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] spin action server unavailable");
        return;
    }

    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(2.0 * M_PI);

    RCLCPP_INFO(get_logger(), "[ALCPlanner] sending spin goal: yaw=2pi");
    alc_rotation_observed_loop_closure_ = false;
    alc_rotation_attempt_active_ = false;

    const std::weak_ptr<ALCPlannerNode> weak_self =
        std::static_pointer_cast<ALCPlannerNode>(shared_from_this());
    rclcpp_action::Client<Spin>::SendGoalOptions options;
    options.goal_response_callback = [weak_self](const auto& handle) {
        const auto self = weak_self.lock();
        if (!self) {
            return;
        }
        if (handle) {
            return;
        }

        self->slam_graph_planner_.onRotationComplete();
        self->alc_rotation_attempt_active_ = false;
        self->last_alc_time_ = self->now();
        RCLCPP_WARN(self->get_logger(), "[ALCPlanner] spin goal rejected");
    };
    options.result_callback = [weak_self](const auto& result) {
        const auto self = weak_self.lock();
        if (!self) {
            return;
        }
        (void)result;
        if (self->alc_rotation_attempt_active_) {
            self->saliency_eval_.observeLoopClosureAttempt(
                self->alc_rotation_observed_loop_closure_);
        }
        self->alc_rotation_attempt_active_ = false;
        self->alc_rotation_observed_loop_closure_ = false;
        self->slam_graph_planner_.onRotationComplete();
        self->last_alc_time_ = self->now();
        RCLCPP_INFO(self->get_logger(), "[ALCPlanner] ALC rotation complete");
    };

    (void)spin_client_->async_send_goal(goal, options);
    alc_rotation_attempt_active_ = true;
}

float ALCPlannerNode::computeCoverageRatio() const {
    return cached_coverage_ratio_;
}

void ALCPlannerNode::logGraphState() const {
    if (!rcutils_logging_logger_is_enabled_for(get_logger().get_name(),
                                               RCUTILS_LOG_SEVERITY_DEBUG)) {
        return;
    }

    std::size_t edge_count = 0;
    for (const auto& edges : graph_.adj) {
        edge_count += edges.size();
    }

    RCLCPP_DEBUG(get_logger(), "[ALCPlanner] graph: %zu nodes, %zu edges",
                 graph_.keyframes.size(), edge_count / 2U);

    std::vector<const Keyframe*> ordered_keyframes;
    ordered_keyframes.reserve(graph_.keyframes.size());
    for (const auto& keyframe : graph_.keyframes) {
        ordered_keyframes.push_back(&keyframe);
    }
    std::sort(ordered_keyframes.begin(), ordered_keyframes.end(),
              [](const Keyframe* lhs, const Keyframe* rhs) {
                  return lhs->node_id < rhs->node_id;
              });

    for (const Keyframe* keyframe_ptr : ordered_keyframes) {
        const Keyframe& keyframe = *keyframe_ptr;
        const auto node_it = graph_.node_to_ix.find(keyframe.node_id);
        const int ix =
            node_it != graph_.node_to_ix.end() ? node_it->second : -1;
        const KeyframeSaliency* saliency =
            (ix >= 0 && ix < static_cast<int>(saliency_state_.keyframes.size()))
                ? &saliency_state_.keyframes[static_cast<std::size_t>(ix)]
                : nullptr;
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] node %d: S_L=%.3f S_G=%.3f "
                     "plc_intrinsic=%.3f words=%zu",
                     keyframe.node_id,
                     saliency ? saliency->saliency_local : 0.0f,
                     saliency ? saliency->saliency_global : 0.0f,
                     saliency ? saliency->plc_intrinsic : 0.0f,
                     keyframe.word_ids.size());
    }

    RCLCPP_DEBUG(get_logger(), "[ALCPlanner] candidates: %zu",
                 candidates_.size());
    for (const auto& candidate : candidates_) {
        const int tau_node_id = nodeIdFromIx(candidate.tau_ix);
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] cand tau_id=%d euclid=%.2f graph_dist=%.2f "
                     "graph_var=%.4f reward_ub=%.4f lighthouse=%d "
                     "cluster_size=%zu",
                     tau_node_id, candidate.euclidean_dist,
                     candidate.graph_dist, candidate.graph_dist_var,
                     candidate.reward_ub,
                     static_cast<int>(candidate.is_lighthouse),
                     candidate.keyframe_ixs.size());
    }

    if (graph_.robot_ix < 0) {
        RCLCPP_DEBUG(get_logger(),
                     "[ALCPlanner] robot_node_id unavailable, skipping graph "
                     "distance logging");
    }
}

int ALCPlannerNode::nodeIdFromIx(const int ix) const {
    if (ix < 0 || ix >= static_cast<int>(graph_.ix_to_node.size())) {
        return -1;
    }
    return graph_.ix_to_node[static_cast<std::size_t>(ix)];
}

}  // namespace alc_planner
