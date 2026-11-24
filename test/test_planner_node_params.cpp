#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>

#include "alc_planner/planner_node.hpp"

namespace alc_planner
{

class ALCPlannerNodeParamsTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        rclcpp::NodeOptions options;
        // Closures are near perfect, so any accumulated yaw drift gives a
        // positive gain and only the position threshold decides.
        options.parameter_overrides(
            {rclcpp::Parameter("cv_L", 1.1),
             rclcpp::Parameter("drift_yaw_var_per_rad", 0.02),
             rclcpp::Parameter("closure_trans_var", 1e-6),
             rclcpp::Parameter("closure_yaw_var", 1e-6),
             rclcpp::Parameter("max_position_std_m", 0.2),
             rclcpp::Parameter("turning_radius_m", 0.7),
             rclcpp::Parameter("map_data_topic", "/test/mapData"),
             rclcpp::Parameter("info_topic", "/test/info"),
             rclcpp::Parameter("map_topic", "/test/map"),
             rclcpp::Parameter("planner_event_topic", "/test/planner_events")});
        node_ = std::make_shared<ALCPlannerNode>(options);
    }

    // Turns 1 rad on the spot and back, then drives straight for `distance`.
    RevisitDecision decide(const float distance) {
        GraphState graph;
        for (int i = 0; i < 4; ++i) {
            Keyframe keyframe;
            keyframe.node_id = i + 1;
            keyframe.word_ids = {2 * i + 1, 2 * i + 2};
            keyframe.pose.position.x() = i == 3 ? distance : 0.0f;
            keyframe.pose.orientation = Eigen::AngleAxisf(
                i == 1 ? 1.0f : 0.0f, Eigen::Vector3f::UnitZ());
            graph.keyframes.push_back(keyframe);
            graph.adj.push_back({});
            if (i > 0) {
                graph.adj[i - 1].push_back({i, false});
                graph.adj[i].push_back({i - 1, false});
            }
        }
        graph.robot_ix = 3;

        nav_msgs::msg::OccupancyGrid map;
        map.info.resolution = 0.5f;
        map.info.width = 20;
        map.info.height = 20;
        map.info.origin.position.x = -5.0;
        map.info.origin.position.y = -5.0;
        map.data.assign(400, 0);

        SaliencyState saliency;
        node_->saliency_eval_.observeWordsRecognized(2);
        node_->saliency_eval_.update(graph, saliency);
        return node_->revisit_selector_.select(graph, saliency, map, {});
    }

    std::shared_ptr<ALCPlannerNode> node_;
};

TEST_F(ALCPlannerNodeParamsTest, TopicOverridesAreUsed) {
    EXPECT_EQ(node_->count_subscribers("/test/mapData"), 1u);
    EXPECT_EQ(node_->count_subscribers("/test/info"), 1u);
    EXPECT_EQ(node_->count_subscribers("/test/map"), 1u);
    EXPECT_EQ(node_->count_subscribers("/test/planner_events"), 1u);
    EXPECT_EQ(node_->count_publishers("/test/planner_events"), 1u);
}

TEST_F(ALCPlannerNodeParamsTest, OverridesReachTheDecision) {
    const auto decision = decide(1.5f);

    // Yaw variance of the 2 rad turned, swung sideways over the drive, plus the
    // default translation drift of 5.4e-4 per metre.
    EXPECT_NEAR(decision.max_position_std,
                std::sqrt(1.5 * 1.5 * 2.0 * 0.02 + 5.4e-4 * 1.5), 1e-3);

    ASSERT_TRUE(decision.best);
    EXPECT_EQ(decision.best->tau_ix, 0);
    // S_L = S_G = 1 and cv_G keeps its default of 3.
    EXPECT_NEAR(decision.best->P_lc, std::tanh(1.1) * std::tanh(3.0), 1e-4);
    // The target lies behind: turn pi towards it and pi back to its view.
    EXPECT_NEAR(decision.best->cost, 1.5 + 0.7 * 2.0 * M_PI, 1e-3);
}

TEST_F(ALCPlannerNodeParamsTest, PositionStdThresholdOverrideIsUsed) {
    const auto decision = decide(0.8f);

    // Above the default threshold of 0.105, below the override of 0.2.
    EXPECT_GT(decision.max_position_std, 0.105f);
    EXPECT_LT(decision.max_position_std, 0.2f);
    EXPECT_FALSE(decision.best);
}

}  // namespace alc_planner
