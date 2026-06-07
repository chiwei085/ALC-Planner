#include <Eigen/Core>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <vector>

#include "alc_planner/bnb_selector.hpp"
#include "alc_planner/map_utils.hpp"
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

int addNode(GraphState& graph, SaliencyState& saliency_state, const int node_id,
            const Eigen::Vector3f& pos, const float plc_intrinsic) {
    const int ix = static_cast<int>(graph.keyframes.size());
    Keyframe keyframe;
    keyframe.node_id = node_id;
    keyframe.pose.position = pos;
    keyframe.pose.orientation = Eigen::Quaternionf::Identity();
    graph.node_to_ix[node_id] = ix;
    graph.ix_to_node.push_back(node_id);
    graph.keyframes.push_back(std::move(keyframe));
    graph.adj.push_back({});

    KeyframeSaliency saliency;
    saliency.plc_intrinsic = plc_intrinsic;
    saliency_state.keyframes.push_back(std::move(saliency));
    return ix;
}

std::pair<GraphState, SaliencyState> makeGraph() {
    GraphState graph;
    SaliencyState saliency_state;
    graph.robot_ix = addNode(graph, saliency_state, 0,
                             Eigen::Vector3f(0.5f, 0.5f, 0.0f), 0.0f);
    return {std::move(graph), std::move(saliency_state)};
}

ALCCandidate makeCandidate(const GraphState& graph, const int tau_ix,
                           const float graph_dist,
                           const std::vector<int>& keyframe_ixs) {
    ALCCandidate candidate;
    candidate.tau_ix = tau_ix;
    candidate.rep_pose = graph.keyframes[static_cast<std::size_t>(tau_ix)].pose;
    candidate.keyframe_ixs = keyframe_ixs;
    candidate.graph_dist = graph_dist;
    candidate.euclidean_dist =
        (graph.keyframes[static_cast<std::size_t>(graph.robot_ix)]
             .pose.position -
         candidate.rep_pose.position)
            .norm();
    return candidate;
}

void fillRewardUBs(std::vector<ALCCandidate>& candidates, const Params& params,
                   const GraphState&,
                   const SaliencyState& saliency_state) {
    RewardEvaluator evaluator(params);
    for (auto& candidate : candidates) {
        evaluator.fillRewardUB(candidate, saliency_state);
    }
}

int nodeIdFromTau(const GraphState& graph, const ALCCandidate& candidate) {
    return graph.ix_to_node[static_cast<std::size_t>(candidate.tau_ix)];
}

}  // namespace

TEST(BNBSelector, EmptyCandidates) {
    Params params;
    const auto [graph, saliency_state] = makeGraph();
    const auto map = buildFreeGrid(10, 10, 1.0f);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select({}, graph, saliency_state, map).has_value());
}

TEST(BNBSelector, NoRobotNode) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    graph.robot_ix = -1;
    const auto map = buildFreeGrid(10, 10, 1.0f);

    BNBSelector selector(params);
    EXPECT_FALSE(selector.select({}, graph, saliency_state, map).has_value());
}

TEST(BNBSelector, SingleCandidateReachable) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    const int tau_ix = addNode(graph, saliency_state, 1,
                               Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.8f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau_ix, 8.0f, {tau_ix})};
    fillRewardUBs(candidates, params, graph, saliency_state);

    const auto map = buildFreeGrid(10, 10, 1.0f);
    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(nodeIdFromTau(graph, *best), 1);
}

TEST(BNBSelector, SingleCandidateGridBlockedFallsBackToEuclidean) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    // Robot at (0.5, 0.5), candidate at (4.5, 4.5) → euclidean ≈ 5.657
    const int tau_ix = addNode(graph, saliency_state, 1,
                               Eigen::Vector3f(4.5f, 4.5f, 0.0f), 0.8f);
    const float graph_dist = 8.0f;
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau_ix, graph_dist, {tau_ix})};
    fillRewardUBs(candidates, params, graph, saliency_state);

    // A full vertical wall at col=2 blocks the grid path. BnB falls back to
    // euclidean_dist so delta_U = graph_dist - euclidean_dist stays positive.
    auto map = buildFreeGrid(10, 10, 1.0f);
    for (int row = 0; row < 10; ++row) {
        setCell(map, row, 2, 100);
    }

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);
    ASSERT_TRUE(best.has_value());
    const float expected_euclidean = std::sqrt(4.0f * 4.0f + 4.0f * 4.0f);
    EXPECT_NEAR(best->map_dist, expected_euclidean, 1e-3f);
}

TEST(BNBSelector, TwoCandidatesHigherRewardWins) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    const int tau1_ix = addNode(graph, saliency_state, 1,
                                Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.8f);
    const int tau2_ix = addNode(graph, saliency_state, 2,
                                Eigen::Vector3f(6.5f, 0.5f, 0.0f), 0.2f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau1_ix, 10.0f, {tau1_ix}),
        makeCandidate(graph, tau2_ix, 8.0f, {tau2_ix}),
    };
    fillRewardUBs(candidates, params, graph, saliency_state);

    const auto map = buildFreeGrid(12, 12, 1.0f);
    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(nodeIdFromTau(graph, *best), 1);
}

TEST(BNBSelector, PruningWorks) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    const int tau1_ix = addNode(graph, saliency_state, 1,
                                Eigen::Vector3f(3.5f, 0.5f, 0.0f), 0.9f);
    const int tau2_ix = addNode(graph, saliency_state, 2,
                                Eigen::Vector3f(8.5f, 8.5f, 0.0f), 0.1f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau1_ix, 9.0f, {tau1_ix}),
        makeCandidate(graph, tau2_ix, 8.0f, {tau2_ix}),
    };
    fillRewardUBs(candidates, params, graph, saliency_state);

    auto map = buildFreeGrid(12, 12, 1.0f);
    setCell(map, 8, 8, 100);

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(nodeIdFromTau(graph, *best), 1);
}

TEST(BNBSelector, AllGridBlockedFallsBackToGraph) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    const int tau1_ix = addNode(graph, saliency_state, 1,
                                Eigen::Vector3f(4.5f, 4.5f, 0.0f), 0.8f);
    const int tau2_ix = addNode(graph, saliency_state, 2,
                                Eigen::Vector3f(5.5f, 5.5f, 0.0f), 0.7f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau1_ix, 8.0f, {tau1_ix}),
        makeCandidate(graph, tau2_ix, 8.0f, {tau2_ix}),
    };
    fillRewardUBs(candidates, params, graph, saliency_state);

    // Full vertical wall at col=2 blocks all grid paths. BnB falls back to
    // graph_dist for both and selects the higher-reward candidate.
    auto map = buildFreeGrid(12, 12, 1.0f);
    for (int row = 0; row < 12; ++row) {
        setCell(map, row, 2, 100);
    }

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);
    ASSERT_TRUE(best.has_value());
    // tau1 has higher plc_intrinsic (0.8 > 0.7) so it wins on reward.
    EXPECT_EQ(nodeIdFromTau(graph, *best), 1);
}

TEST(BNBSelector, MixedReachability) {
    Params params;
    auto [graph, saliency_state] = makeGraph();
    const int tau1_ix = addNode(graph, saliency_state, 1,
                                Eigen::Vector3f(4.5f, 0.5f, 0.0f), 0.5f);
    const int tau2_ix = addNode(graph, saliency_state, 2,
                                Eigen::Vector3f(2.5f, 0.5f, 0.0f), 0.9f);
    const int tau3_ix = addNode(graph, saliency_state, 3,
                                Eigen::Vector3f(8.5f, 8.5f, 0.0f), 0.8f);
    std::vector<ALCCandidate> candidates = {
        makeCandidate(graph, tau1_ix, 7.0f, {tau1_ix}),
        makeCandidate(graph, tau2_ix, 9.0f, {tau2_ix}),
        makeCandidate(graph, tau3_ix, 9.0f, {tau3_ix}),
    };
    fillRewardUBs(candidates, params, graph, saliency_state);

    auto map = buildFreeGrid(12, 12, 1.0f);
    setCell(map, 8, 8, 100);

    BNBSelector selector(params);
    const auto best = selector.select(candidates, graph, saliency_state, map);

    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(nodeIdFromTau(graph, *best), 2);
}

TEST(BNBSelector, SaliencyOverlayUsesMaxForSharedCell) {
    auto [graph, saliency_state] = makeGraph();
    const int ix1 = addNode(graph, saliency_state, 1,
                            Eigen::Vector3f(1.1f, 1.1f, 0.0f), 0.0f);
    const int ix2 = addNode(graph, saliency_state, 2,
                            Eigen::Vector3f(1.2f, 1.2f, 0.0f), 0.0f);
    saliency_state.keyframes[static_cast<std::size_t>(ix1)].saliency_local =
        0.3f;
    saliency_state.keyframes[static_cast<std::size_t>(ix2)].saliency_local =
        0.8f;

    const auto map = buildFreeGrid(4, 4, 1.0f);
    std::vector<float> overlay;
    buildSaliencyOverlay(graph, saliency_state, map, overlay);

    const GridCell cell = toCell(
        graph.keyframes[static_cast<std::size_t>(ix1)].pose.position, map);
    const int index = toIndex(cell, static_cast<int>(map.info.width));
    EXPECT_NEAR(overlay[static_cast<std::size_t>(index)], 0.8f, 1e-4f);
}

TEST(BNBSelector, SaliencyOverlaySkipsOutOfBoundsKeyframes) {
    auto [graph, saliency_state] = makeGraph();
    const int ix = addNode(graph, saliency_state, 1,
                           Eigen::Vector3f(100.0f, 100.0f, 0.0f), 0.0f);
    saliency_state.keyframes[static_cast<std::size_t>(ix)].saliency_local =
        0.9f;

    const auto map = buildFreeGrid(4, 4, 1.0f);
    std::vector<float> overlay;
    buildSaliencyOverlay(graph, saliency_state, map, overlay);

    EXPECT_EQ(std::count_if(overlay.begin(), overlay.end(),
                            [](const float value) { return value > 0.0f; }),
              0);
}

}  // namespace alc_planner
