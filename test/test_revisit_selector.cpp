#include <gtest/gtest.h>

#include <cmath>

#include "alc_planner/revisit_selector.hpp"

namespace alc_planner
{
namespace
{

struct World
{
    GraphState graph;
    SaliencyState saliency;
    nav_msgs::msg::OccupancyGrid map;

    // Keyframes along a path, chained by odometry; the last is the robot.
    void add(const float x, const float y, const float yaw) {
        Keyframe k;
        k.node_id = static_cast<int>(graph.keyframes.size());
        k.pose.position = {x, y, 0.0f};
        k.pose.orientation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        graph.keyframes.push_back(k);
        graph.adj.push_back({});
        KeyframeSaliency s;
        s.saliency_local = 0.9f;
        s.plc_intrinsic = 0.6f;
        saliency.keyframes.push_back(s);
        const int ix = static_cast<int>(graph.keyframes.size()) - 1;
        if (ix > 0) {
            link(ix - 1, ix, false);
        }
        graph.robot_ix = ix;
    }
    void link(const int a, const int b, const bool closure) {
        graph.adj[static_cast<std::size_t>(a)].push_back({b, closure});
        graph.adj[static_cast<std::size_t>(b)].push_back({a, closure});
    }
    void freeMap(const float half) {
        map.info.resolution = 0.5f;
        const auto cells = static_cast<unsigned>(2.0f * half / 0.5f);
        map.info.width = cells;
        map.info.height = cells;
        map.info.origin.position.x = -half;
        map.info.origin.position.y = -half;
        map.data.assign(cells * cells, 0);
    }
    RevisitDecision select(const std::vector<Pose6f>& excluded = {}) const {
        return RevisitSelector(Params{}).select(graph, saliency, map, excluded);
    }
};

// Square loop of the given side that ends one step short of its start.
World loop(const float side) {
    World w;
    w.freeMap(1.5f * side + 5.0f);
    const int steps = std::max(4, static_cast<int>(side / 0.5f));
    for (int edge = 0; edge < 4; ++edge) {
        const float yaw = static_cast<float>(edge) * static_cast<float>(M_PI_2);
        const float x0 = (edge == 1 || edge == 2) ? side : 0.0f;
        const float y0 = (edge >= 2) ? side : 0.0f;
        for (int i = 0; i < steps - (edge == 3 ? 1 : 0); ++i) {
            const float s = side * static_cast<float>(i) / steps;
            w.add(x0 + s * std::cos(yaw), y0 + s * std::sin(yaw), yaw);
        }
    }
    return w;
}

TEST(RevisitSelector, LongLoopTriggersWhateverItsLength) {
    for (const float side : {10.0f, 30.0f, 100.0f}) {
        const auto decision = loop(side).select();
        ASSERT_TRUE(decision.best) << side;
        EXPECT_GT(decision.best->gain, 0.0f);
    }
}

// Driving away for 100 m makes the start very uncertain, but nothing here
// overlaps its map: there is no reason to go back.
TEST(RevisitSelector, DrivingAwayStaysSilent) {
    World w;
    w.freeMap(120.0f);
    for (int i = 0; i <= 200; ++i) {
        w.add(0.5f * static_cast<float>(i), 0.0f, 0.0f);
        ASSERT_FALSE(w.select().best) << i;
    }
}

TEST(RevisitSelector, LongerLoopsAreMoreUncertainAndGainMore) {
    const auto small = loop(10.0f).select();
    const auto large = loop(50.0f).select();
    EXPECT_GT(large.max_position_std, small.max_position_std);
    EXPECT_GT(large.best->gain, small.best->gain);
}

TEST(RevisitSelector, ShortPathDoesNotTrigger) {
    World w;
    w.freeMap(10.0f);
    for (int i = 0; i < 5; ++i) {
        w.add(0.5f * static_cast<float>(i), 0.0f, 0.0f);
    }
    EXPECT_FALSE(w.select().best);
}

TEST(RevisitSelector, RecentClosureRemovesTheReason) {
    auto w = loop(10.0f);
    const float before = w.select().max_position_std;
    w.link(0, w.graph.robot_ix, true);
    EXPECT_LT(w.select().max_position_std, before);
}

// Out and back along a corridor: the robot stands where it once was but faces
// the other way. Passive closures cannot happen, so it must still be a target.
TEST(RevisitSelector, CorridorReturnTargetsTheOppositeView) {
    World w;
    w.freeMap(30.0f);
    for (int i = 0; i <= 40; ++i) {
        w.add(0.5f * static_cast<float>(i), 0.0f, 0.0f);
    }
    for (int i = 39; i >= 0; --i) {
        w.add(0.5f * static_cast<float>(i), 0.0f, static_cast<float>(M_PI));
    }
    const auto decision = w.select();
    ASSERT_TRUE(decision.best);
    EXPECT_LE(decision.best->tau_ix, 40);
    EXPECT_GT(decision.best->rep_pose.forward().x(), 0.9f);
}

// Turning costs yaw, and yaw costs metres.
TEST(RevisitSelector, TurningIsWorseThanDrivingStraight) {
    World straight;
    World turning;
    straight.freeMap(30.0f);
    turning.freeMap(30.0f);
    for (int i = 0; i <= 8; ++i) {
        const auto x = 0.5f * static_cast<float>(i);
        straight.add(x, 0.0f, 0.0f);
        turning.add(x, 0.0f, i % 2 ? 0.3f : -0.3f);
    }
    EXPECT_GT(turning.select().max_position_std,
              straight.select().max_position_std);
}

TEST(RevisitSelector, SkipsTargetsThatAlreadyFailed) {
    const auto w = loop(10.0f);
    const auto first = w.select();
    ASSERT_TRUE(first.best);
    const auto second = w.select({first.best->rep_pose});
    ASSERT_TRUE(second.best);
    EXPECT_NE(second.best->tau_ix, first.best->tau_ix);
}

TEST(RevisitSelector, UnreachableTargetIsNotChosen) {
    auto w = loop(10.0f);
    // Wall everything except the robot's own cell column.
    w.map.data.assign(w.map.data.size(), 100);
    const auto decision = w.select();
    if (decision.best) {
        EXPECT_LT(decision.best->euclidean_dist, 1.0f);
    }
}

TEST(RevisitSelector, DisconnectedKeyframesAreIgnored) {
    auto w = loop(10.0f);
    const auto before = w.select().max_position_std;
    Keyframe lost;
    lost.pose.position = {500.0f, 0.0f, 0.0f};
    const int robot = w.graph.robot_ix;
    w.graph.keyframes.push_back(lost);
    w.graph.adj.push_back({});
    w.saliency.keyframes.push_back({});
    w.graph.robot_ix = robot;
    EXPECT_FLOAT_EQ(w.select().max_position_std, before);
}

}  // namespace
}  // namespace alc_planner
