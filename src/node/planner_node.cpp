#include "alc_planner/planner_node.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rcutils/logging.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
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

// Half a second without a Nav2 goal: an exploration goal sent just before
// alc_started has been accepted and cancelled by then.
constexpr int kNavIdleTicks = 5;
constexpr double kDefaultNavigationTimeoutSec = 120.0;
constexpr const char* kDefaultPlannerEventTopic = "/planner/events";

std::optional<std::string> jsonStringField(const nlohmann::json& payload,
                                           const std::string& key) {
    if (!payload.contains(key) || !payload[key].is_string()) {
        return std::nullopt;
    }
    return payload[key].get<std::string>();
}

}  // namespace

ALCPlannerNode::ALCPlannerNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("alc_planner", opts),
      params_(declarePlannerParams()),
      saliency_eval_(params_),
      revisit_selector_(params_) {
    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "/spin");
    event_pub_ =
        create_publisher<std_msgs::msg::String>(planner_event_topic_, 10);

    sub_map_data_ = create_subscription<rtabmap_msgs::msg::MapData>(
        declare_parameter<std::string>("map_data_topic", "/rtabmap/mapData"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onMapData, this, std::placeholders::_1));
    sub_info_ = create_subscription<rtabmap_msgs::msg::Info>(
        declare_parameter<std::string>("info_topic", "/rtabmap/info"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onInfo, this, std::placeholders::_1));
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        declare_parameter<std::string>("map_topic", "/map"),
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ALCPlannerNode::onMap, this, std::placeholders::_1));
    nav_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const action_msgs::msg::GoalStatusArray& msg) {
            // 1 accepted, 2 executing, 3 canceling.
            nav_goals_active_ = std::any_of(
                msg.status_list.begin(), msg.status_list.end(),
                [](const auto& s) { return s.status >= 1 && s.status <= 3; });
        });
    sub_planner_event_ = create_subscription<std_msgs::msg::String>(
        planner_event_topic_, 10,
        std::bind(&ALCPlannerNode::onPlannerEvent, this,
                  std::placeholders::_1));
}

Params ALCPlannerNode::declarePlannerParams() {
    Params params;

    const auto declare_float = [this](const std::string& name,
                                      const float default_value) {
        return static_cast<float>(declare_parameter<double>(
            name, static_cast<double>(default_value)));
    };
    for (const auto& [name, value] :
         std::initializer_list<std::pair<const char*, float*>>{
             {"cv_L", &params.cv_L},
             {"cv_G", &params.cv_G},
             {"cs", &params.cs},
             {"cs_lighthouse", &params.cs_lighthouse},
             {"d_min_lighthouse", &params.d_min_lighthouse},
             {"drift_trans_var_per_m", &params.drift_trans_var_per_m},
             {"drift_yaw_var_per_rad", &params.drift_yaw_var_per_rad},
             {"closure_trans_var", &params.closure_trans_var},
             {"closure_yaw_var", &params.closure_yaw_var},
             {"max_position_std_m", &params.max_position_std_m},
             {"turning_radius_m", &params.turning_radius_m},
             {"arrival_xy_m", &params.arrival_xy_m},
             {"arrival_yaw_rad", &params.arrival_yaw_rad}}) {
        *value = declare_float(name, *value);
    }

    navigation_timeout_sec_ = declare_parameter<double>(
        "navigation_timeout_sec", kDefaultNavigationTimeoutSec);
    spin_at_target_ = declare_parameter<bool>("spin_at_target", true);
    // Empty uses the default tree. The R2 passes the straight-tail one so the
    // car arrives facing the keyframe's view.
    nav_behavior_tree_ =
        declare_parameter<std::string>("nav_behavior_tree", "");
    planner_event_topic_ = declare_parameter<std::string>(
        "planner_event_topic", kDefaultPlannerEventTopic);

    return params;
}

void ALCPlannerNode::onMapData(
    const rtabmap_msgs::msg::MapData::SharedPtr msg) {
    if (!msg) {
        return;
    }

    ingestNodes(*msg);
    // RTAB-Map publishes info before the mapData that adds its ref node.
    updateRobotIx();
    ingestLinks(*msg);
    saliency_eval_.update(graph_, saliency_state_);
    checkLighthouse();
    maybeTrigger();

    last_map_data_stamp_ = msg->header.stamp;
    logGraphState();
}

void ALCPlannerNode::resolveAttempt() {
    if (!attempt_) {
        return;
    }
    if (attempt_->reached) {
        saliency_eval_.observeLoopClosureAttempt(attempt_->closure);
    }
    if (!attempt_->closure) {
        failed_targets_.push_back(attempt_->target);
    }
    attempt_.reset();
}

void ALCPlannerNode::maybeTrigger() {
    // A moving robot keeps its goal: cancelling it costs more than waiting.
    if (exploration_moving_ || !occupancy_map_ ||
        slam_graph_planner_.state() != PlannerState::EVALUATING) {
        return;
    }
    resolveAttempt();
    const auto decision = revisit_selector_.select(
        graph_, saliency_state_, *occupancy_map_, failed_targets_);
    publishEvaluation(decision);
    if (decision.best && slam_graph_planner_.startNavigation()) {
        sendNavGoal(*decision.best);
    }
}

void ALCPlannerNode::onInfo(const rtabmap_msgs::msg::Info::SharedPtr msg) {
    if (!msg) {
        return;
    }

    publishLoopClosure(*msg);
    latest_ref_id_ = msg->ref_id;
    updateRobotIx();
    if (msg->loop_closure_id > 0) {
        RCLCPP_INFO(get_logger(), "[ALCPlanner] loop closure detected: id=%d",
                    msg->loop_closure_id);
    }

    if (attempt_ && attempt_->reached && msg->loop_closure_id > 0) {
        attempt_->closure = true;
    }
}

void ALCPlannerNode::updateRobotIx() {
    const auto it = graph_.node_to_ix.find(latest_ref_id_);
    graph_.robot_ix = it != graph_.node_to_ix.end() ? it->second : -1;
}

void ALCPlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    occupancy_map_ = msg;
}

void ALCPlannerNode::onPlannerEvent(
    const std_msgs::msg::String::SharedPtr msg) {
    if (!msg) {
        return;
    }
    try {
        const auto payload = nlohmann::json::parse(msg->data);
        const std::string event =
            jsonStringField(payload, "event").value_or(std::string{});
        if (event.rfind("alc_", 0) == 0) {
            return;
        }
        if (event == "goal_sent" || event == "subgoal_sent") {
            // A route's first goal finds the robot at rest, so a revisit can
            // take its place without stopping anything.
            if (payload.contains("index") &&
                static_cast<int>(payload[std::string("index")]) == 0) {
                exploration_moving_ = false;
                maybeTrigger();
            }
            exploration_moving_ = true;
        }
        else if (event == "recovery_tree_started" ||
                 event == "observation_turn_started") {
            exploration_moving_ = true;
        }
        else if (event == "route_completed") {
            exploration_moving_ = false;
            maybeTrigger();
        }
    }
    catch (const nlohmann::json::parse_error&) {
    }
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
        // RTAB-Map gives features outside the dictionary negative ids, which
        // repeat in every node and say nothing about the place.
        keyframe.word_ids.clear();
        std::copy_if(node_msg.word_id_keys.begin(), node_msg.word_id_keys.end(),
                     std::back_inserter(keyframe.word_ids),
                     [](const int32_t id) { return id > 0; });
        saliency_eval_.observeWordsRecognized(
            static_cast<int>(keyframe.word_ids.size()));
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
        // RTAB-Map: 0 neighbor and 6 merged neighbor are odometry.
        const bool closure = link.type != 0 && link.type != 6;
        graph_.adj[static_cast<std::size_t>(from_ix)].push_back(
            {to_ix, closure});
        graph_.adj[static_cast<std::size_t>(to_ix)].push_back(
            {from_ix, closure});
    }

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
    attempt_ = Attempt{target.rep_pose};
    if (!nav_client_) {
        (void)slam_graph_planner_.onNavigationResult(false);
        publishAlcFinished("navigation", false, "nav_client_unavailable");
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] navigate_to_pose action client unavailable");
        return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
        (void)slam_graph_planner_.onNavigationResult(false);
        publishAlcFinished("navigation", false, "nav_server_unavailable");
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
    goal_orientation.normalize();
    goal.pose.pose.orientation.x = goal_orientation.x();
    goal.pose.pose.orientation.y = goal_orientation.y();
    goal.pose.pose.orientation.z = goal_orientation.z();
    goal.pose.pose.orientation.w = goal_orientation.w();
    goal.behavior_tree = nav_behavior_tree_;

    RCLCPP_INFO(get_logger(),
                "[ALCPlanner] sending nav goal: tau_id=%d x=%.2f y=%.2f",
                nodeIdFromIx(target.tau_ix), target.rep_pose.position.x(),
                target.rep_pose.position.y());
    publishAlcStarted(target);

    const std::weak_ptr<ALCPlannerNode> weak_self =
        std::static_pointer_cast<ALCPlannerNode>(shared_from_this());
    const std::uint64_t attempt = ++nav_attempt_;
    nav_goal_handle_.reset();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [weak_self, attempt](const auto& handle) {
        const auto self = weak_self.lock();
        if (!self || self->nav_attempt_ != attempt) {
            return;
        }
        if (handle) {
            self->nav_goal_handle_ = handle;
            self->startNavigationTimeout();
            return;
        }

        self->cancelNavigationTimeout();
        (void)self->slam_graph_planner_.onNavigationResult(false);
        self->publishAlcFinished("navigation", false, "goal_rejected");
        RCLCPP_WARN(self->get_logger(),
                    "[ALCPlanner] navigation goal rejected");
    };
    options.result_callback = [weak_self, attempt](const auto& result) {
        const auto self = weak_self.lock();
        if (!self || self->nav_attempt_ != attempt) {
            return;
        }
        self->nav_goal_handle_.reset();
        self->cancelNavigationTimeout();
        const bool success =
            result.code == rclcpp_action::ResultCode::SUCCEEDED;
        const bool should_spin =
            self->slam_graph_planner_.onNavigationResult(success);
        if (should_spin) {
            if (self->attempt_) {
                self->attempt_->reached = true;
            }
            if (self->spin_at_target_) {
                self->sendSpinGoal();
                return;
            }
            self->slam_graph_planner_.onRotationComplete();
            self->publishAlcFinished("navigation", true, "target_reached");
            return;
        }

        if (!success) {
            self->publishAlcFinished("navigation", false, "navigation_failed");
            RCLCPP_WARN(self->get_logger(),
                        "[ALCPlanner] navigation to ALC target failed");
        }
    };

    // Nav2 rejects a preemption that changes the behavior tree, so the
    // exploration goal that alc_started cancels must be gone first.
    startNavigationTimeout();
    nav_idle_ticks_ = 0;
    nav_idle_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), [weak_self, attempt, goal, options]() {
            const auto self = weak_self.lock();
            if (!self) {
                return;
            }
            if (self->nav_attempt_ != attempt) {
                self->nav_idle_timer_->cancel();
                return;
            }
            self->nav_idle_ticks_ =
                self->nav_goals_active_ ? 0 : self->nav_idle_ticks_ + 1;
            if (self->nav_idle_ticks_ < kNavIdleTicks) {
                return;
            }
            self->nav_idle_timer_->cancel();
            self->nav_client_->async_send_goal(goal, options);
        });
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

            // Abandon the goal for real: cancel it in Nav2 and drop its
            // callbacks, so the robot stops chasing it and its late result
            // cannot touch a newer attempt.
            if (self->nav_goal_handle_) {
                (void)self->nav_client_->async_cancel_goal(
                    self->nav_goal_handle_);
                self->nav_goal_handle_.reset();
            }
            ++self->nav_attempt_;
            (void)self->slam_graph_planner_.onNavigationResult(false);
            self->publishAlcFinished("navigation", false, "navigation_timeout");
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
        publishAlcFinished("rotation", false, "spin_client_unavailable");
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] spin action client unavailable");
        return;
    }
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(0))) {
        slam_graph_planner_.onRotationComplete();
        publishAlcFinished("rotation", false, "spin_server_unavailable");
        RCLCPP_WARN(get_logger(),
                    "[ALCPlanner] spin action server unavailable");
        return;
    }

    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(2.0 * M_PI);
    // Nav2 treats a zero allowance as "never time out".
    goal.time_allowance =
        rclcpp::Duration::from_seconds(std::max(0.0, navigation_timeout_sec_));

    RCLCPP_INFO(get_logger(), "[ALCPlanner] sending spin goal: yaw=2pi");

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
        self->publishAlcFinished("rotation", false, "spin_rejected");
        RCLCPP_WARN(self->get_logger(), "[ALCPlanner] spin goal rejected");
    };
    options.result_callback = [weak_self](const auto& result) {
        const auto self = weak_self.lock();
        if (!self) {
            return;
        }
        (void)result;
        self->slam_graph_planner_.onRotationComplete();
        const bool success =
            result.code == rclcpp_action::ResultCode::SUCCEEDED;
        self->publishAlcFinished("rotation", success,
                                 success ? "spin_completed" : "spin_failed");
        RCLCPP_INFO(self->get_logger(), "[ALCPlanner] ALC rotation complete");
    };

    (void)spin_client_->async_send_goal(goal, options);
}

void ALCPlannerNode::publishAlcStarted(const ALCCandidate& target) {
    if (!event_pub_) {
        return;
    }
    nlohmann::json payload = {
        {"event", "alc_started"},
        {"time_sec", now().seconds()},
        {"phase", "navigation"},
        {"tau_id", nodeIdFromIx(target.tau_ix)},
        {"goal_xy", nlohmann::json::array({target.rep_pose.position.x(),
                                           target.rep_pose.position.y()})},
        {"P_lc", target.P_lc},
        {"gain_nats", target.gain},
        {"position_std_m", target.position_std},
        {"max_position_std_m", params_.max_position_std_m},
        {"map_dist", target.map_dist},
        {"cost_m", target.cost},
        {"score", target.score},
    };
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    event_pub_->publish(msg);
}

// Every evaluation is logged, so a silent ALC shows how far it was from acting.
void ALCPlannerNode::publishEvaluation(const RevisitDecision& decision) {
    nlohmann::json payload = {
        {"event", "alc_evaluated"},
        {"time_sec", now().seconds()},
        {"largest_position_std_m", decision.max_position_std},
        {"max_position_std_m", params_.max_position_std_m},
        {"tau_id", nullptr},
    };
    if (decision.best) {
        payload[std::string("tau_id")] = nodeIdFromIx(decision.best->tau_ix);
        payload[std::string("P_lc")] = decision.best->P_lc;
        payload[std::string("gain_nats")] = decision.best->gain;
        payload[std::string("cost_m")] = decision.best->cost;
        payload[std::string("score")] = decision.best->score;
    }
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    event_pub_->publish(msg);
}

// What each closure actually changed: the correction is RTAB-Map's own
// measurement, the only direct evidence of what a revisit was worth.
void ALCPlannerNode::publishLoopClosure(const rtabmap_msgs::msg::Info& info) {
    if (info.loop_closure_id <= 0 && info.proximity_detection_id <= 0) {
        return;
    }
    const auto stat = [&info](const std::string& key) -> nlohmann::json {
        const auto count =
            std::min(info.stats_keys.size(), info.stats_values.size());
        for (std::size_t i = 0; i < count; ++i) {
            if (info.stats_keys[i] == key) {
                return info.stats_values[i];
            }
        }
        return nullptr;
    };
    const nlohmann::json payload = {
        {"event", "loop_closure"},
        {"time_sec", now().seconds()},
        {"ref_id", info.ref_id},
        {"loop_closure_id", info.loop_closure_id},
        {"proximity_id", info.proximity_detection_id},
        {"correction_m", stat("Loop/Odom_correction_norm/m")},
        {"correction_deg", stat("Loop/Odom_correction_angle/deg")},
        {"map_to_odom_m", stat("Loop/MapToOdom_norm/m")},
        {"map_to_odom_deg", stat("Loop/MapToOdom_yaw/deg")},
        {"inliers", stat("Loop/Visual_inliers/")},
        {"during_alc", slam_graph_planner_.state() != PlannerState::EVALUATING},
    };
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    event_pub_->publish(msg);
}

void ALCPlannerNode::publishAlcFinished(const std::string& phase,
                                        const bool success,
                                        const std::string& reason) {
    if (!event_pub_) {
        return;
    }
    nlohmann::json payload = {
        {"event", "alc_finished"}, {"time_sec", now().seconds()},
        {"phase", phase},          {"success", success},
        {"reason", reason},
    };
    std_msgs::msg::String msg;
    msg.data = payload.dump();
    event_pub_->publish(msg);
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
