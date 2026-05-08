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

void addNode(
    GraphState& graph, const int id, const Eigen::Vector3f& pos, const float sl,
    const float sg,
    const Eigen::Quaternionf& orient = Eigen::Quaternionf::Identity()) {
    Keyframe keyframe;
    keyframe.node_id = id;
    keyframe.pose.position = pos;
    keyframe.pose.orientation = orient;
    keyframe.saliency_local = sl;
    keyframe.saliency_global = sg;
    keyframe.plc_intrinsic = std::tanh(3.0f * sl) * std::tanh(3.0f * sg);
    graph.keyframes.emplace(id, keyframe);
    graph.adj[id];
}

void addBiEdge(GraphState& graph, const int lhs_id, const int rhs_id,
               const float dist) {
    graph.adj[lhs_id].push_back({rhs_id, dist});
    graph.adj[rhs_id].push_back({lhs_id, dist});
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

const ALCCandidate* findCandidateByTauId(
    const std::vector<ALCCandidate>& candidates, const int tau_id) {
    const auto it = std::find_if(candidates.begin(), candidates.end(),
                                 [tau_id](const ALCCandidate& candidate) {
                                     return candidate.tau_id == tau_id;
                                 });
    return it == candidates.end() ? nullptr : &(*it);
}

}  // namespace

TEST(CandidateBuilder, NoRobotNodeReturnsEmpty) {
    GraphState graph = makeGraph();
    addNode(graph, 1, {1.0f, 0.0f, 0.0f}, 0.8f, 0.8f);

    CandidateBuilder builder(makeParams());
    EXPECT_TRUE(builder.build(graph).empty());
}

TEST(CandidateBuilder, FilterExcludesLowSaliency) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {3.0f, 0.0f, 0.0f}, 0.1f, 0.7f);
    addNode(graph, 2, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, 3, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 5.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    addBiEdge(graph, 0, 3, 6.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ids.size(), 2U);
    EXPECT_EQ(candidates.front().tau_id, 2);
}

TEST(CandidateBuilder, FilterExcludesSmallGraphDist) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {3.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addNode(graph, 2, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, 3, {5.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addBiEdge(graph, 0, 1, 1.0f);
    addBiEdge(graph, 0, 2, 4.0f);
    addBiEdge(graph, 0, 3, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ids.size(), 2U);
    EXPECT_EQ(candidates.front().tau_id, 3);
}

TEST(CandidateBuilder, FilterExcludesFarEuclidean) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {20.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, 2, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, 3, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 20.0f);
    addBiEdge(graph, 0, 2, 4.0f);
    addBiEdge(graph, 0, 3, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ids.size(), 2U);
    EXPECT_EQ(candidates.front().tau_id, 3);
}

TEST(CandidateBuilder, DBSCANMergesCloseNodes) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().keyframe_ids.size(), 2U);
}

TEST(CandidateBuilder, DBSCANDropsNoise) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 4.0f);

    CandidateBuilder builder(makeParams());
    EXPECT_TRUE(builder.build(graph).empty());
}

TEST(CandidateBuilder, DBSCANSeparatesDistantClusters) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, 3, {12.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addNode(graph, 4, {13.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);
    addBiEdge(graph, 0, 3, 12.0f);
    addBiEdge(graph, 0, 4, 13.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 2U);
    EXPECT_NE(nullptr, findCandidateByTauId(candidates, 2));
    EXPECT_NE(nullptr, findCandidateByTauId(candidates, 4));
}

TEST(CandidateBuilder, RepSelectsHigherPLCIntrinsic) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.7f, 0.3f);
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().tau_id, 2);
}

TEST(CandidateBuilder, RepViewingDirectionFilter) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.9f, 0.9f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.6f, 0.6f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().tau_id, 2);
}

TEST(CandidateBuilder, RepFallbackWhenAllFilteredOut) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.9f, 0.9f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f,
            Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(M_PI),
                                                 Eigen::Vector3f::UnitZ())));
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().tau_id, 1);
}

TEST(CandidateBuilder, LighthouseStateCarriesToCandidate) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.9f, 0.9f);
    graph.keyframes.at(2).is_lighthouse = true;
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_TRUE(candidates.front().is_lighthouse);
}

TEST(CandidateBuilder, CandidateDistancesAreCorrect) {
    GraphState graph = makeGraph();
    graph.robot_node_id = 0;
    addNode(graph, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    addNode(graph, 1, {4.0f, 0.0f, 0.0f}, 0.8f, 0.8f);
    addNode(graph, 2, {5.0f, 0.0f, 0.0f}, 0.7f, 0.7f);
    addBiEdge(graph, 0, 1, 4.0f);
    addBiEdge(graph, 0, 2, 5.0f);

    CandidateBuilder builder(makeParams());
    const auto candidates = builder.build(graph);

    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().tau_id, 1);
    EXPECT_NEAR(candidates.front().euclidean_dist, 4.0f, 1e-4f);
    EXPECT_NEAR(candidates.front().graph_dist, 4.0f, 1e-4f);
}

}  // namespace alc_planner
