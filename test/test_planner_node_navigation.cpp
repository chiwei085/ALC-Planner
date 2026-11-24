#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "alc_planner/planner_node.hpp"

namespace alc_planner
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Spin = nav2_msgs::action::Spin;
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
            {rclcpp::Parameter("navigation_timeout_sec", 1.5),
             rclcpp::Parameter("nav_behavior_tree", "view.xml"),
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
        spin_server_ = rclcpp_action::create_server<Spin>(
            fake_, "/spin",
            [this](const auto&, const auto&) {
                ++spin_requests_;
                return rclcpp_action::GoalResponse::REJECT;
            },
            [](const auto&) { return rclcpp_action::CancelResponse::REJECT; },
            [](const auto&) {});
        events_ = fake_->create_subscription<std_msgs::msg::String>(
            "/test_alc_nav/events", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("alc_finished") != std::string::npos) {
                    last_finished_ = msg->data;
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
        target.rep_pose.orientation = Eigen::Quaternionf::Identity();
        ASSERT_TRUE(node_->slam_graph_planner_.startNavigation());
        node_->sendNavGoal(target);
    }

    bool serverReady() const {
        return node_->nav_client_->action_server_is_ready() &&
               node_->spin_client_->action_server_is_ready();
    }
    PlannerState state() const { return node_->slam_graph_planner_.state(); }
    bool timeoutArmed() const { return node_->nav_timeout_timer_ != nullptr; }
    int robotIx() const { return node_->graph_.robot_ix; }
    std::size_t wordCount() const {
        return node_->graph_.keyframes.at(0).word_ids.size();
    }
    // A robot that drove 20 m out, winding, and came back beside its start.
    void planCandidate() {
        for (int id = 1; id <= 41; ++id) {
            Keyframe keyframe;
            keyframe.node_id = id;
            keyframe.word_ids = {1, 2, 3};
            keyframe.pose.position.x() =
                static_cast<float>(id <= 21 ? id - 1 : 41 - id);
            keyframe.pose.position.y() = id <= 21 ? 0.0f : 1.0f;
            keyframe.pose.orientation = Eigen::AngleAxisf(
                id % 2 ? 0.4f : -0.4f, Eigen::Vector3f::UnitZ());
            node_->graph_.node_to_ix[id] = id - 1;
            node_->graph_.ix_to_node.push_back(id);
            node_->graph_.keyframes.push_back(keyframe);
            node_->graph_.adj.push_back({});
            KeyframeSaliency saliency;
            saliency.saliency_local = 0.9f;
            saliency.plc_intrinsic = 0.8f;
            node_->saliency_state_.keyframes.push_back(saliency);
        }
        for (int i = 1; i < 41; ++i) {
            node_->graph_.adj[i - 1].push_back({i, false});
            node_->graph_.adj[i].push_back({i - 1, false});
        }
        node_->graph_.robot_ix = 40;
        auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        map->info.resolution = 1.0f;
        map->info.width = 60;
        map->info.height = 20;
        map->info.origin.position.x = -5.0;
        map->info.origin.position.y = -10.0;
        map->data.assign(1200, 0);
        node_->occupancy_map_ = map;
    }
    std::size_t failedTargets() const { return node_->failed_targets_.size(); }
    void sendEvent(const std::string& name, const int index = 1) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = "{\"event\":\"" + name +
                    "\",\"index\":" + std::to_string(index) + "}";
        node_->onPlannerEvent(msg);
    }
    void maybeTrigger() { node_->maybeTrigger(); }
    void sendInfo(const int ref_id, const int loop_closure_id = 0) {
        auto info = std::make_shared<rtabmap_msgs::msg::Info>();
        info->ref_id = ref_id;
        info->loop_closure_id = loop_closure_id;
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
    bool goalHandleHeld() const { return node_->nav_goal_handle_ != nullptr; }

    std::shared_ptr<ALCPlannerNode> node_;
    std::shared_ptr<rclcpp::Node> fake_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
    rclcpp_action::Server<Spin>::SharedPtr spin_server_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::vector<std::shared_ptr<ServerGoal>> goals_;
    int cancel_requests_ = 0;
    int spin_requests_ = 0;
    int finished_ = 0;
    std::string last_finished_;
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

    // Attempt B waits while A is still in Nav2: a goal with another behavior
    // tree would be rejected there.
    startAttempt();
    spinUntil([] { return false; }, 0.7);
    EXPECT_EQ(goals_.size(), 1u);

    // A's late result must not fail B, disarm B's timeout, or finish again.
    goals_[0]->canceled(std::make_shared<NavigateToPose::Result>());
    ASSERT_TRUE(spinUntil([this] { return goals_.size() == 2u; }, 2.0));
    EXPECT_EQ(state(), PlannerState::NAVIGATING_TO_ALC);
    EXPECT_TRUE(timeoutArmed());
    EXPECT_EQ(finished_, 1);
}

TEST_F(ALCPlannerNodeNavigationTest, ArrivalFinishesWithoutSpin) {
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    startAttempt();
    ASSERT_TRUE(spinUntil([this] { return goalHandleHeld(); }, 2.0));
    EXPECT_EQ(goals_[0]->get_goal()->behavior_tree, "view.xml");
    goals_[0]->succeed(std::make_shared<NavigateToPose::Result>());
    ASSERT_TRUE(spinUntil([this] { return finished_ == 1; }, 0.8));
    for (const char* field : {"\"phase\":\"navigation\"", "\"success\":true",
                              "\"reason\":\"target_reached\""}) {
        EXPECT_NE(last_finished_.find(field), std::string::npos)
            << last_finished_;
    }
    EXPECT_EQ(spin_requests_, 0);
    EXPECT_EQ(state(), PlannerState::EVALUATING);
}

TEST_F(ALCPlannerNodeNavigationTest, FirstGoalOfARouteIsADecisionPoint) {
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    planCandidate();
    sendEvent("recovery_tree_started");
    sendEvent("goal_sent", 1);  // later waypoint: the robot is driving
    EXPECT_EQ(state(), PlannerState::EVALUATING);
    sendEvent("goal_sent", 0);
    EXPECT_EQ(state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST_F(ALCPlannerNodeNavigationTest, TargetWithoutClosureIsNotTriedAgain) {
    ASSERT_TRUE(spinUntil([this] { return serverReady(); }, 5.0));
    planCandidate();
    maybeTrigger();
    ASSERT_TRUE(spinUntil([this] { return goals_.size() == 1u; }, 2.0));
    ASSERT_TRUE(spinUntil([this] { return goalHandleHeld(); }, 2.0));
    goals_[0]->abort(std::make_shared<NavigateToPose::Result>());
    ASSERT_TRUE(spinUntil([this] { return finished_ == 1; }, 2.0));
    sendInfo(41, 5);  // a passive closure is no success of this attempt
    maybeTrigger();
    EXPECT_EQ(failedTargets(), 1u);
    ASSERT_TRUE(spinUntil([this] { return goals_.size() == 2u; }, 2.0));
    EXPECT_NE(goals_[0]->get_goal()->pose.pose.position.x,
              goals_[1]->get_goal()->pose.pose.position.x);
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

}  // namespace alc_planner
