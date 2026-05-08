#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>

#include <memory>

#include "alc_planner/candidate_builder.hpp"
#include "alc_planner/saliency_evaluator.hpp"
#include "alc_planner/types.hpp"
#include "alc_planner/uncertainty_metrics.hpp"

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

    void ingestNodes(const rtabmap_msgs::msg::MapData& msg);
    void ingestLinks(const rtabmap_msgs::msg::MapData& msg);
    void checkLighthouse();
    void logGraphState() const;

    Params params_;
    SaliencyEvaluator saliency_eval_;
    CandidateBuilder candidate_builder_;
    GraphState graph_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_;
    std::vector<ALCCandidate> candidates_;
    Pose6f last_lighthouse_pose_;
    bool has_lighthouse_ = false;
    rclcpp::Time last_map_data_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr sub_map_data_;
    rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr sub_info_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
};

}  // namespace alc_planner
