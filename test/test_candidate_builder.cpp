#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "alc_planner/candidate_builder.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{
namespace
{

GraphState makeGraph() {
    return GraphState{};
}

int addNode(GraphState& graph, SaliencyState& saliency_state, const int node_id,
            const Eigen::Vector3f& pos, const float sl, const float sg,
            const Eigen::Quaternionf& orient = Eigen::Quaternionf::Identity()) {
    const int ix = static_cast<int>(graph.keyframes.size());
    Keyframe keyframe;
    keyframe.node_id = node_id;
    keyframe.pose.position = pos;
    keyframe.pose.orientation = orient;
    graph.node_to_ix[node_id] = ix;
    graph.ix_to_node.push_back(node_id);
    graph.keyframes.push_back(std::move(keyframe));
    graph.adj.push_back({});

    KeyframeSaliency saliency;
    saliency.saliency_local = sl;
    saliency.saliency_global = sg;
    saliency.plc_intrinsic = std::tanh(3.0f * sl) * std::tanh(3.0f * sg);
    saliency_state.keyframes.push_back(std::move(saliency));
    return ix;
}

void addBiEdge(GraphState& graph, const int lhs_node_id, const int rhs_node_id,
               const float dist) {
    const int lhs_ix = graph.node_to_ix.at(lhs_node_id);
    const int rhs_ix = graph.node_to_ix.at(rhs_node_id);
    graph.adj[static_cast<std::size_t>(lhs_ix)].push_back({rhs_ix, dist});
    graph.adj[static_cast<std::size_t>(rhs_ix)].push_back({lhs_ix, dist});
}

Params makeParams() {
    Params params;
    params.cE = 15.0f;
    params.cG = 3.0f;
    params.cs = 0.3f;
    params.eps_dbscan = 1.5f;
    params.min_pts = 2;
    return params;
}

const ALCCandidate* findCandidateByTauNodeId(
    const GraphState& graph, const std::vector<ALCCandidate>& candidates,
    const int tau_node_id) {
    const auto it =
        std::find_if(candidates.begin(), candidates.end(),
                     [&graph, tau_node_id](const ALCCandidate& candidate) {
                         return graph.ix_to_node[static_cast<std::size_t>(
                                    candidate.tau_ix)] == tau_node_id;
                     });
    return it == candidates.end() ? nullptr : &(*it);
}

int nodeIdFromTau(const GraphState& graph, const ALCCandidate& candidate) {
    return graph.ix_to_node[static_cast<std::size_t>(candidate.tau_ix)];
}

}  // namespace

TEST(CandidateBuilder, NoRobotNodeReturnsEmpty) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    addNode(graph, saliency_state, 1, {1.0f, 0.0f, 0.0f}, 0.8f, 0.8f);

    CandidateBuilder builder(makeParams());
    EXPECT_TRUE(builder.build(graph, saliency_state).empty());
}

TEST(CandidateBuilder, FilterExcludesLowSaliency) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {3.0f, 0.0f, 0.0f}, 0.1f, 0.7f);
    addNode(graph, saliency_state, 2, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, saliency_state, 3, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 5.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    addBiEdge(graph, 0, 3, 6.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ixs.size(), 2U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 2);
}

TEST(CandidateBuilder, FilterExcludesSmallGraphDist) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {3.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addNode(graph, saliency_state, 2, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, saliency_state, 3, {5.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addBiEdge(graph, 0, 1, 1.0f);
    addBiEdge(graph, 0, 2, 4.0f);
    addBiEdge(graph, 0, 3, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ixs.size(), 2U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 3);
}

TEST(CandidateBuilder, FilterExcludesDisconnectedNodes) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addBiEdge(graph, 1, 2, 1.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    EXPECT_TRUE(builder.build(graph, saliency_state).empty());
}

TEST(CandidateBuilder, FilterExcludesFarEuclidean) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {20.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, saliency_state, 2, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, saliency_state, 3, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 20.0f);
    addBiEdge(graph, 0, 2, 4.0f);
    addBiEdge(graph, 0, 3, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ixs.size(), 2U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 3);
}

TEST(CandidateBuilder, DBSCANMergesCloseNodes) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ixs.size(), 2U);
}

TEST(CandidateBuilder, DBSCANDropsNoise) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 4.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    EXPECT_TRUE(builder.build(graph, saliency_state).empty());
}

TEST(CandidateBuilder, DBSCANSeparatesDistantClusters) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, saliency_state, 3, {12.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addNode(graph, saliency_state, 4, {13.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    addBiEdge(graph, 0, 3, 12.0f);
    addBiEdge(graph, 0, 4, 13.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 2U);
    EXPECT_NE(nullptr, findCandidateByTauNodeId(graph, candidates, 2));
    EXPECT_NE(nullptr, findCandidateByTauNodeId(graph, candidates, 4));
}

TEST(CandidateBuilder, RepSelectsHigherPLCIntrinsic) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.3f);
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 2);
}

TEST(CandidateBuilder, RepViewingDirectionFilter) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.9f, 0.9f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 2);
}

TEST(CandidateBuilder, RepFallbackWhenAllFilteredOut) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.9f, 0.9f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 1);
}

TEST(CandidateBuilder, LighthouseStateCarriesToCandidate) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    const int ix2 =
        addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    saliency_state.keyframes[static_cast<std::size_t>(ix2)].is_lighthouse =
        true;
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_TRUE(candidates.front().is_lighthouse);
}

TEST(CandidateBuilder, CandidateDistancesAreCorrect) {
    GraphState graph = makeGraph();
    SaliencyState saliency_state;
    graph.robot_ix =
        addNode(graph, saliency_state, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, saliency_state, 1, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, saliency_state, 2, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    graph.version = 1;

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph, saliency_state);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(nodeIdFromTau(graph, candidates.front()), 1);
    EXPECT_NEAR(candidates.front().euclidean_dist, 4.0f, 1e-4f);
    EXPECT_NEAR(candidates.front().graph_dist, 4.0f, 1e-4f);
}

}  // namespace alc_planner
