#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "alc_planner/planner_node.hpp"

namespace alc_planner
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ServerGoal = rclcpp_action::ServerGoalHandle<NavigateToPose>;

// Fake Nav2: accepts every goal and never finishes one on its own, so the
// test decides when (and whether) each goal's result arrives.
class ALCPlannerNodeNavigationTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("navigation_timeout_sec", 1.0),
             rclcpp::Parameter("use_approach_heading", false),
             rclcpp::Parameter("planner_event_topic", "/test_alc_nav/events")});
        node_ = std::make_shared<ALCPlannerNode>(options);
        fake_ = std::make_shared<rclcpp::Node>("fake_nav2");
        server_ = rclcpp_action::create_server<NavigateToPose>(
            fake_, "/navigate_to_pose",
            [](const auto&, const auto&) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const auto&) {
                ++cancel_requests_;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<ServerGoal> goal) {
                goals_.push_back(goal);
            });
        events_ = fake_->create_subscription<std_msgs::msg::String>(
            "/test_alc_nav/events", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("alc_finished") != std::string::npos) {
                    ++finished_;
                }
            });
        executor_.add_node(node_);
        executor_.add_node(fake_);
    }

    bool spinUntil(const std::function<bool()>& done, const double sec) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::duration<double>(sec);
        while (!done() && std::chrono::steady_clock::now() < end) {
            executor_.spin_some(std::chrono::milliseconds(10));
        }
        return done();
    }

    // Puts the planner into NAVIGATING_TO_ALC and sends the goal, the same
    // two calls onMapData makes when an ALC triggers.
    void startAttempt() {
        ALCCandidate target;
        target.tau_ix = 0;
        target.P_lc = 1.0f;
        target.map_dist = 10.0f;
        target.reward = 10.0f;
        target.rep_pose.orientation = Eigen::Quaternionf::Identity();
        ASSERT_TRUE(node_->slam_graph_planner_
                        .onEvaluationComplete(
                            target, std::numeric_limits<double>::infinity(),
                            0.0f, 100, 0.0f)
                        .has_value());
        node_->sendNavGoal(target);
    }

    bool serverReady() const {
        return node_->nav_client_->action_server_is_ready();
    }
    PlannerState state() const { return node_->slam_graph_planner_.state(); }
    bool timeoutArmed() const { return node_->nav_timeout_timer_ != nullptr; }
    bool goalHandleHeld() const { return node_->nav_goal_handle_ != nullptr; }

    std::shared_ptr<ALCPlannerNode> node_;
    std::shared_ptr<rclcpp::Node> fake_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::vector<std::shared_ptr<ServerGoal>> goals_;
    int cancel_requests_ = 0;
    int finished_ = 0;
};

TEST_F(ALCPlannerNodeNavigationTest,
       TimeoutCancelsGoalAndIgnoresItsLateResult) {
    // sendNavGoal does not wait for discovery, so let it happen first.
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    startAttempt();
    ASSERT_TRUE(spinUntil([this] { return goalHandleHeld(); }, 2.0));
    ASSERT_EQ(goals_.size(), 1u);

    // Attempt A times out: Nav2 is told to cancel it, and ALC finishes once.
    ASSERT_TRUE(spinUntil([this] { return finished_ == 1; }, 3.0));
    EXPECT_TRUE(spinUntil([this] { return cancel_requests_ == 1; }, 1.0));
    EXPECT_EQ(state(), PlannerState::EVALUATING);

    // Attempt B starts before A's result has come back.
    startAttempt();
    ASSERT_TRUE(spinUntil([this] { return goals_.size() == 2u; }, 2.0));
    ASSERT_TRUE(spinUntil([this] { return goalHandleHeld(); }, 2.0));

    // A's late result must not fail B, disarm B's timeout, or finish again.
    goals_[0]->canceled(std::make_shared<NavigateToPose::Result>());
    spinUntil([] { return false; }, 0.3);
    EXPECT_EQ(state(), PlannerState::NAVIGATING_TO_ALC);
    EXPECT_TRUE(timeoutArmed());
    EXPECT_EQ(finished_, 1);
}

}  // namespace alc_planner
