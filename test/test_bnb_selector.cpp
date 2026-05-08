#include <Eigen/Core>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>

#include "alc_planner/bnb_selector.hpp"
#include "alc_planner/reward_evaluator.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{
namespace
{

nav_msgs::msg::OccupancyGrid buildFreeGrid(const int width, const int height,
                                           const float resolution) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = width;
    map.info.height = height;
    map.info.resolution = resolution;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(width * height), 0);
    return map;
}

void setCell(nav_msgs::msg::OccupancyGrid& map, const int row, const int col,
             const int8_t value) {
    const int width = static_cast<int>(map.info.width);
    map.data[static_cast<std::size_t>(row * width + col)] = value;
}

void addNode(GraphState& graph, const int id, const Eigen::Vector3f& pos,
             const float plc_intrinsic) {
    Keyframe keyframe;
    keyframe.node_id = id;
    keyframe.pose.position = pos;
    keyframe.pose.orientation = Eigen::Quaternionf::Identity();
    keyframe.plc_intrinsic = plc_intrinsic;
    graph.keyframes.emplace(id, keyframe);
}

GraphState makeGraph() {
    GraphState graph;
    graph.robot_node_id = 0;
    addNode(graph, 0, Eigen::Vector3f(0.5f, 0.5f, 0.0f), 0.0f);
    return graph;
}

ALCCandidate makeCandidate(const GraphState& graph, const int tau_id,
                           const float graph_dist,
                           const std::vector<int>& keyframe_ids) {
    ALCCandidate candidate;
    candidate.tau_id = tau_id;
    candidate.rep_pose = graph.keyframes.at(tau_id).pose;
    candidate.keyframe_ids = keyframe_ids;
    candidate.graph_dist = graph_dist;
    candidate.euclidean_dist =
        (graph.keyframes.at(0).pose.position - candidate.rep_pose.position)
            .norm();
    return candidate;
}

void fillRewardUBs(std::vector<ALCCandidate>& candidates, const Params& params,
                   const GraphState& graph) {
    RewardEvaluator evaluator(params);
    for (auto& candidate : candidates) {
        evaluator.fillRewardUB(candidate, graph);
    }
}

}  // namespace

TEST(BNBSelector, EmptyCandidates) {
    Params params;
    const GraphState graph = makeGraph();
    const auto map = buildFreeGrid(10, 10, 1.0f);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select({}, graph, map).has_value());
}

TEST(BNBSelector, NoRobotNode) {
    Params params;
    GraphState graph = makeGraph();
    graph.robot_node_id = -1;
    const auto map = buildFreeGrid(10, 10, 1.0f);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select({}, graph, map).has_value());
}

TEST(BNBSelector, SingleCandidateReachable) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.8f);
    std::vector<ALCCandidate> candidates = {makeCandidate(graph, 1, 8.0f, {1})};
    fillRewardUBs(candidates, params, graph);

    const auto map = buildFreeGrid(10, 10, 1.0f);
    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->tau_id, 1);
}

TEST(BNBSelector, SingleCandidateUnreachable) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(4.5f, 4.5f, 0.0f), 0.8f);
    std::vector<ALCCandidate> candidates = {makeCandidate(graph, 1, 8.0f, {1})};
    fillRewardUBs(candidates, params, graph);

    auto map = buildFreeGrid(10, 10, 1.0f);
    setCell(map, 4, 4, 100);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select(candidates, graph, map).has_value());
}

TEST(BNBSelector, TwoCandidatesHigherRewardWins) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.8f);
    addNode(graph, 2, Eigen::Vector3f(6.5f, 0.5f, 0.0f), 0.2f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, 1, 10.0f, {1}),
        makeCandidate(graph, 2, 8.0f, {2}),
    };
    fillRewardUBs(candidates, params, graph);

    const auto map = buildFreeGrid(12, 12, 1.0f);
    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->tau_id, 1);
}

TEST(BNBSelector, PruningWorks) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(3.5f, 0.5f, 0.0f), 0.9f);
    addNode(graph, 2, Eigen::Vector3f(8.5f, 8.5f, 0.0f), 0.1f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, 1, 9.0f, {1}),
        makeCandidate(graph, 2, 8.0f, {2}),
    };
    fillRewardUBs(candidates, params, graph);

    auto map = buildFreeGrid(12, 12, 1.0f);
    setCell(map, 8, 8, 100);

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->tau_id, 1);
}

TEST(BNBSelector, AllUnreachable) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(4.5f, 4.5f, 0.0f), 0.8f);
    addNode(graph, 2, Eigen::Vector3f(5.5f, 5.5f, 0.0f), 0.7f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, 1, 8.0f, {1}),
        makeCandidate(graph, 2, 8.0f, {2}),
    };
    fillRewardUBs(candidates, params, graph);

    auto map = buildFreeGrid(12, 12, 1.0f);
    setCell(map, 4, 4, 100);
    setCell(map, 5, 5, 100);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select(candidates, graph, map).has_value());
}

TEST(BNBSelector, MixedReachability) {
    Params params;
    GraphState graph = makeGraph();
    addNode(graph, 1, Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.5f);
    addNode(graph, 2, Eigen::Vector3f(2.5f, 0.5f, 0.0f), 0.9f);
    addNode(graph, 3, Eigen::Vector3f(8.5f, 8.5f, 0.0f), 0.8f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, 1, 7.0f, {1}),
        makeCandidate(graph, 2, 9.0f, {2}),
        makeCandidate(graph, 3, 9.0f, {3}),
    };
    fillRewardUBs(candidates, params, graph);

    auto map = buildFreeGrid(12, 12, 1.0f);
    setCell(map, 8, 8, 100);

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->tau_id, 2);
}

}  // namespace alc_planner
