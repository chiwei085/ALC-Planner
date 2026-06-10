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
         rclcpp::Parameter("cv_G", 1.2),
         rclcpp::Parameter("cs_lighthouse", 0.2),
         rclcpp::Parameter("d_min_lighthouse", 2.0),
         rclcpp::Parameter("cl", 6.0),
         rclcpp::Parameter("ct", 0.2),
         rclcpp::Parameter("cE", 8.0),
         rclcpp::Parameter("cG", 2.0),
         rclcpp::Parameter("cs", 0.15),
         rclcpp::Parameter("eps_dbscan", 1.0),
         rclcpp::Parameter("min_pts", 3),
         rclcpp::Parameter("theta_max", 0.25),
         rclcpp::Parameter("lambda_decay", 0.03),
         rclcpp::Parameter("alpha_cov", 0.4),
         rclcpp::Parameter("tau_min_revisit", 4),
         rclcpp::Parameter("plc_min_revisit", 0.02),
         rclcpp::Parameter("map_dist_min_revisit", 0.75),
         rclcpp::Parameter("use_variance_uncertainty", true),
         rclcpp::Parameter("navigation_timeout_sec", 37.0),
         rclcpp::Parameter("use_approach_heading", false),
         rclcpp::Parameter("planner_event_topic", "/test/planner_events")});

    const auto node = std::make_shared<ALCPlannerNode>(options);

    EXPECT_NEAR(node->get_parameter("cv_L").as_double(), 1.1, 1e-6);
    EXPECT_NEAR(node->get_parameter("cv_G").as_double(), 1.2, 1e-6);
    EXPECT_NEAR(node->get_parameter("cs_lighthouse").as_double(), 0.2, 1e-6);
    EXPECT_NEAR(node->get_parameter("d_min_lighthouse").as_double(), 2.0, 1e-6);
    EXPECT_NEAR(node->get_parameter("cl").as_double(), 6.0, 1e-6);
    EXPECT_NEAR(node->get_parameter("ct").as_double(), 0.2, 1e-6);
    EXPECT_NEAR(node->get_parameter("cE").as_double(), 8.0, 1e-6);
    EXPECT_NEAR(node->get_parameter("cG").as_double(), 2.0, 1e-6);
    EXPECT_NEAR(node->get_parameter("cs").as_double(), 0.15, 1e-6);
    EXPECT_NEAR(node->get_parameter("eps_dbscan").as_double(), 1.0, 1e-6);
    EXPECT_EQ(node->get_parameter("min_pts").as_int(), 3);
    EXPECT_NEAR(node->get_parameter("theta_max").as_double(), 0.25, 1e-6);
    EXPECT_NEAR(node->get_parameter("lambda_decay").as_double(), 0.03, 1e-6);
    EXPECT_NEAR(node->get_parameter("alpha_cov").as_double(), 0.4, 1e-6);
    EXPECT_EQ(node->get_parameter("tau_min_revisit").as_int(), 4);
    EXPECT_NEAR(node->get_parameter("plc_min_revisit").as_double(), 0.02, 1e-6);
    EXPECT_NEAR(node->get_parameter("map_dist_min_revisit").as_double(), 0.75,
                1e-6);
    EXPECT_TRUE(node->get_parameter("use_variance_uncertainty").as_bool());
    EXPECT_NEAR(node->get_parameter("navigation_timeout_sec").as_double(), 37.0,
                1e-6);
    EXPECT_FALSE(node->get_parameter("use_approach_heading").as_bool());
    EXPECT_EQ(node->get_parameter("planner_event_topic").as_string(),
              "/test/planner_events");

    rclcpp::shutdown();
}

}  // namespace
}  // namespace alc_planner
