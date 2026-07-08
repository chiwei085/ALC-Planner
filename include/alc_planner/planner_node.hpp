#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <string>
#include <vector>

#include "alc_planner/bnb_selector.hpp"
#include "alc_planner/candidate_builder.hpp"
#include "alc_planner/reward_evaluator.hpp"
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
    void onMapData(const rtabmap_msgs::msg::MapData::SharedPtr msg);
    void onInfo(const rtabmap_msgs::msg::Info::SharedPtr msg);
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void onLocalizationPose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void onPlannerEvent(const std_msgs::msg::String::SharedPtr msg);

    void ingestNodes(const rtabmap_msgs::msg::MapData& msg);
    void ingestLinks(const rtabmap_msgs::msg::MapData& msg);
    void checkLighthouse();
    void sendNavGoal(const ALCCandidate& target);
    void sendSpinGoal();
    Params declarePlannerParams();
    void startNavigationTimeout();
    void cancelNavigationTimeout();
    void publishAlcStarted(const ALCCandidate& target);
    void publishAlcFinished(const std::string& phase, bool success,
                            const std::string& reason);
    void pruneDegradationEvents(double now_sec);
    bool degradedExplorationActive(double now_sec);
    float computeCoverageRatio() const;
    void logGraphState() const;
    int nodeIdFromIx(int ix) const;

    double navigation_timeout_sec_ = 0.0;
    double degraded_exploration_alc_bonus_ = 0.0;
    double degraded_exploration_window_sec_ = 60.0;
    int degraded_exploration_min_events_ = 3;
    bool use_approach_heading_ = true;
    std::string planner_event_topic_;
    Params params_;
    SaliencyEvaluator saliency_eval_;
    CandidateBuilder candidate_builder_;
    RewardEvaluator reward_evaluator_;
    BNBSelector bnb_selector_;
    SLAMGraphPlanner slam_graph_planner_;
    GraphState graph_;
    SaliencyState saliency_state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_;
    float cached_coverage_ratio_ = 0.5f;
    float current_pose_uncertainty_lambda_ = 0.0f;
    std::vector<ALCCandidate> candidates_;
    std::optional<ALCCandidate> best_candidate_;
    Pose6f last_lighthouse_pose_;
    bool has_lighthouse_ = false;
    bool alc_rotation_attempt_active_ = false;
    bool alc_rotation_observed_loop_closure_ = false;
    rclcpp::Time last_alc_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_map_data_stamp_{0, 0, RCL_ROS_TIME};

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using Spin = nav2_msgs::action::Spin;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<Spin>::SharedPtr spin_client_;
    rclcpp::TimerBase::SharedPtr nav_timeout_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr sub_map_data_;
    rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr sub_info_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
        SharedPtr sub_localization_pose_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_planner_event_;
    std::vector<double> degraded_exploration_event_times_;
};

}  // namespace alc_planner
