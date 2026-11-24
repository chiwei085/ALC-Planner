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
             rclcpp::Parameter("spin_at_target", false),
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
    int robotIx() const { return node_->graph_.robot_ix; }
    std::size_t wordCount() const {
        return node_->graph_.keyframes.at(0).word_ids.size();
    }
    void planCandidate() {
        ALCCandidate c;
        c.tau_ix = 0;
        c.P_lc = 1.0f;
        c.map_dist = 10.0f;
        c.reward = 10.0f;
        c.rep_pose.orientation = Eigen::Quaternionf::Identity();
        node_->best_candidate_ = c;
        node_->graph_.robot_ix = 100;
    }
    void sendEvent(const std::string& name) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = "{\"event\":\"" + name + "\"}";
        node_->onPlannerEvent(msg);
    }
    void maybeTrigger() { node_->maybeTrigger(); }
    std::vector<float> viewOverlap(const float keyframe_yaw,
                                   const float robot_x) {
        node_->graph_.keyframes.resize(1);
        node_->graph_.keyframes[0].pose.orientation =
            Eigen::AngleAxisf(keyframe_yaw, Eigen::Vector3f::UnitZ());
        ALCCandidate c;
        c.keyframe_ixs = {0};
        c.rep_pose = Pose6f{};
        Pose6f robot;
        robot.position.x() = robot_x;
        node_->use_approach_heading_ = true;
        node_->setViewOverlap(c, robot);
        return c.view_overlap;
    }
    void sendInfo(const int ref_id) {
        auto info = std::make_shared<rtabmap_msgs::msg::Info>();
        info->ref_id = ref_id;
        node_->onInfo(info);
    }
    void sendMapData(const int node_id) {
        auto map = std::make_shared<rtabmap_msgs::msg::MapData>();
        rtabmap_msgs::msg::Node node;
        node.id = node_id;
        node.pose.orientation.w = 1.0;
        node.word_id_keys = {1, 2, 3, 4, -1, -2};
        map->nodes.push_back(node);
        map->graph.poses_id = {node_id};
        map->graph.poses = {node.pose};
        node_->onMapData(map);
    }
    double secSinceAlc() const {
        return (node_->now() - node_->last_alc_time_).seconds();
    }
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

TEST_F(ALCPlannerNodeNavigationTest, ArrivalFinishesWithoutSpin) {
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    startAttempt();
    ASSERT_TRUE(spinUntil([this] { return goalHandleHeld(); }, 2.0));
    goals_[0]->succeed(std::make_shared<NavigateToPose::Result>());
    ASSERT_TRUE(spinUntil([this] { return finished_ == 1; }, 0.8));
    EXPECT_EQ(state(), PlannerState::EVALUATING);
    EXPECT_LT(secSinceAlc(), 5.0);
}

TEST_F(ALCPlannerNodeNavigationTest, FailedAttemptRestartsTriggerDecay) {
    startAttempt();  // no discovery yet, or goal left to time out
    ASSERT_TRUE(
        spinUntil([this] { return state() == PlannerState::EVALUATING; }, 4.0));
    EXPECT_LT(secSinceAlc(), 5.0);
}

TEST_F(ALCPlannerNodeNavigationTest, InfoBeforeMapDataLocatesTheRobot) {
    sendInfo(7);
    EXPECT_EQ(robotIx(), -1);
    sendMapData(7);
    EXPECT_EQ(robotIx(), 0);
    // Words outside the dictionary (negative ids) do not count.
    EXPECT_EQ(wordCount(), 4u);
}

TEST_F(ALCPlannerNodeNavigationTest, WaitsForTheRouteToFinish) {
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    planCandidate();
    sendEvent("goal_sent");
    maybeTrigger();
    EXPECT_EQ(state(), PlannerState::EVALUATING);
    sendEvent("recovery_tree_started");
    maybeTrigger();
    EXPECT_EQ(state(), PlannerState::EVALUATING);
    sendEvent("route_completed");
    EXPECT_EQ(state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST_F(ALCPlannerNodeNavigationTest, ViewOverlapComparesArrivalWithKeyframe) {
    // Arriving from -x means facing +x (yaw 0).
    EXPECT_FLOAT_EQ(viewOverlap(0.0f, -2.0f).at(0), 1.0f);
    EXPECT_NEAR(viewOverlap(0.5f, -2.0f).at(0), 0.5f, 1e-4f);
    EXPECT_FLOAT_EQ(viewOverlap(2.6f, -2.0f).at(0), 0.0f);
}

}  // namespace alc_planner
