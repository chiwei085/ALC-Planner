#pragma once

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "alc_planner/revisit_selector.hpp"
#include "alc_planner/saliency_evaluator.hpp"
#include "alc_planner/slam_graph_planner.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{

class ALCPlannerNode : public rclcpp::Node
{
public:
    explicit ALCPlannerNode(
        const rclcpp::NodeOptions& opts = rclcpp::NodeOptions{});

private:
    friend class ALCPlannerNodeNavigationTest;
    void onMapData(const rtabmap_msgs::msg::MapData::SharedPtr msg);
    void onInfo(const rtabmap_msgs::msg::Info::SharedPtr msg);
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onPlannerEvent(const std_msgs::msg::String::SharedPtr msg);

    void ingestNodes(const rtabmap_msgs::msg::MapData& msg);
    void ingestLinks(const rtabmap_msgs::msg::MapData& msg);
    void updateRobotIx();
    void maybeTrigger();
    void resolveAttempt();
    void publishEvaluation(const RevisitDecision& decision);
    void checkLighthouse();
    void sendNavGoal(const ALCCandidate& target);
    void sendSpinGoal();
    Params declarePlannerParams();
    void startNavigationTimeout();
    void cancelNavigationTimeout();
    void publishAlcStarted(const ALCCandidate& target);
    void publishLoopClosure(const rtabmap_msgs::msg::Info& info);
    void publishAlcFinished(const std::string& phase, bool success,
                            const std::string& reason);
    void logGraphState() const;
    int nodeIdFromIx(int ix) const;

    double navigation_timeout_sec_ = 0.0;
    bool spin_at_target_ = true;
    std::string nav_behavior_tree_;
    int latest_ref_id_ = 0;
    // The last target driven to, until the next evaluation shows whether a
    // closure came of it.
    struct Attempt
    {
        Pose6f target;
        bool reached = false;
        bool closure = false;
    };
    std::optional<Attempt> attempt_;
    std::vector<Pose6f> failed_targets_;
    // Set by exploration events; false when nothing else drives the robot.
    bool exploration_moving_ = false;
    std::string planner_event_topic_;
    Params params_;
    SaliencyEvaluator saliency_eval_;
    RevisitSelector revisit_selector_;
    SLAMGraphPlanner slam_graph_planner_;
    GraphState graph_;
    SaliencyState saliency_state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_;
    Pose6f last_lighthouse_pose_;
    bool has_lighthouse_ = false;
    rclcpp::Time last_map_data_stamp_{0, 0, RCL_ROS_TIME};

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using Spin = nav2_msgs::action::Spin;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;
    rclcpp::TimerBase::SharedPtr nav_timeout_timer_;
    // Each ALC navigation goal gets an id; callbacks from an older goal (one
    // that timed out or was preempted) are ignored.
    std::uint64_t nav_attempt_ = 0;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_handle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr sub_map_data_;
    rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr sub_info_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_planner_event_;
};

}  // namespace alc_planner
