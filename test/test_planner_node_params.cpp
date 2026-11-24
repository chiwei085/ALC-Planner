#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "alc_planner/planner_node.hpp"

namespace alc_planner
{
namespace
{

TEST(ALCPlannerNode, DeclaresAndUsesParameterOverrides) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("cv_L", 1.1),
         rclcpp::Parameter("drift_yaw_var_per_rad", 0.02),
         rclcpp::Parameter("max_position_std_m", 0.2),
         rclcpp::Parameter("turning_radius_m", 0.7),
         rclcpp::Parameter("navigation_timeout_sec", 37.0),
         rclcpp::Parameter("nav_behavior_tree", "/tmp/tree.xml"),
         rclcpp::Parameter("map_data_topic", "/test/mapData"),
         rclcpp::Parameter("info_topic", "/test/info"),
         rclcpp::Parameter("map_topic", "/test/map"),
         rclcpp::Parameter("planner_event_topic", "/test/planner_events")});

    const auto node = std::make_shared<ALCPlannerNode>(options);

    EXPECT_NEAR(node->get_parameter("cv_L").as_double(), 1.1, 1e-6);
    EXPECT_NEAR(node->get_parameter("drift_yaw_var_per_rad").as_double(), 0.02,
                1e-6);
    EXPECT_NEAR(node->get_parameter("max_position_std_m").as_double(), 0.2,
                1e-6);
    EXPECT_NEAR(node->get_parameter("turning_radius_m").as_double(), 0.7, 1e-6);
    EXPECT_NEAR(node->get_parameter("navigation_timeout_sec").as_double(), 37.0,
                1e-6);
    EXPECT_EQ(node->get_parameter("nav_behavior_tree").as_string(),
              "/tmp/tree.xml");
    EXPECT_EQ(node->get_parameter("planner_event_topic").as_string(),
              "/test/planner_events");
    EXPECT_EQ(node->count_subscribers("/test/mapData"), 1u);
    EXPECT_EQ(node->count_subscribers("/test/info"), 1u);
    EXPECT_EQ(node->count_subscribers("/test/map"), 1u);

    rclcpp::shutdown();
}

}  // namespace
}  // namespace alc_planner
