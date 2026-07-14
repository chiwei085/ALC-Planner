#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "alc_planner/types.hpp"
#include "alc_planner/uncertainty_metrics.hpp"

namespace alc_planner
{
namespace
{

int addNode(GraphState& graph, const int node_id) {
    const int ix = static_cast<int>(graph.keyframes.size());
    Keyframe keyframe;
    keyframe.node_id = node_id;
    graph.node_to_ix[node_id] = ix;
    graph.ix_to_node.push_back(node_id);
    graph.keyframes.push_back(std::move(keyframe));
    graph.adj.push_back({});
    return ix;
}

void addDirectedEdge(GraphState& graph, const int from_node_id,
                     const int to_node_id, const float dist,
                     const float variance = -1.0f) {
    const int from_ix = graph.node_to_ix.at(from_node_id);
    const int to_ix = graph.node_to_ix.at(to_node_id);
    graph.adj[static_cast<std::size_t>(from_ix)].push_back(
        {to_ix, dist, variance >= 0.0f ? variance : dist});
}

std::array<double, 36> makeCovariance(const double x_var, const double y_var,
                                      const double yaw_var) {
    std::array<double, 36> covariance{};
    covariance[0] = x_var;
    covariance[7] = y_var;
    covariance[35] = yaw_var;
    return covariance;
}

}  // namespace

TEST(UncertaintyMetrics, DijkstraKnownGraph) {
    GraphState graph;
    for (int node_id = 0; node_id <= 4; ++node_id) {
        addNode(graph, node_id);
    }

    addDirectedEdge(graph, 0, 1, 1.0f);
    addDirectedEdge(graph, 1, 2, 2.0f);
    addDirectedEdge(graph, 2, 3, 0.5f);
    addDirectedEdge(graph, 1, 4, 1.0f);
    addDirectedEdge(graph, 4, 3, 3.0f);

    const auto dist_map =
        UncertaintyMetrics::dijkstraAll(graph, graph.node_to_ix.at(0));
    ASSERT_EQ(dist_map.size(), 5U);
    EXPECT_NEAR(dist_map[static_cast<std::size_t>(graph.node_to_ix.at(1))],
                1.0f, 1e-4f);
    EXPECT_NEAR(dist_map[static_cast<std::size_t>(graph.node_to_ix.at(2))],
                3.0f, 1e-4f);
    EXPECT_NEAR(dist_map[static_cast<std::size_t>(graph.node_to_ix.at(4))],
                2.0f, 1e-4f);
    EXPECT_NEAR(UncertaintyMetrics::graphDist(graph, graph.node_to_ix.at(0),
                                              graph.node_to_ix.at(3)),
                3.5f, 1e-4f);
}

TEST(UncertaintyMetrics, UnreachableReturnsInf) {
    GraphState graph;
    addNode(graph, 0);
    addNode(graph, 1);
    addNode(graph, 2);
    addDirectedEdge(graph, 0, 1, 1.0f);

    EXPECT_TRUE(std::isinf(UncertaintyMetrics::graphDist(
        graph, graph.node_to_ix.at(0), graph.node_to_ix.at(2))));
}

TEST(UncertaintyMetrics, LoopClosureEdgeShortcutsPath) {
    GraphState graph;
    for (int node_id = 0; node_id <= 3; ++node_id) {
        addNode(graph, node_id);
    }

    addDirectedEdge(graph, 0, 1, 2.0f);
    addDirectedEdge(graph, 1, 2, 2.0f);
    addDirectedEdge(graph, 2, 3, 2.0f);
    addDirectedEdge(graph, 0, 3, 1.2f);

    EXPECT_NEAR(UncertaintyMetrics::graphDist(graph, graph.node_to_ix.at(0),
                                              graph.node_to_ix.at(3)),
                1.2f, 1e-4f);
}

TEST(UncertaintyMetrics, SourceNotInGraphReturnsInf) {
    GraphState graph;
    addNode(graph, 1);
    addNode(graph, 2);
    addDirectedEdge(graph, 1, 2, 1.0f);

    const auto dist = UncertaintyMetrics::dijkstraAll(graph, 99);
    EXPECT_EQ(dist.size(), 2U);
    EXPECT_TRUE(
        std::isinf(dist[static_cast<std::size_t>(graph.node_to_ix.at(1))]));
    EXPECT_TRUE(std::isinf(
        UncertaintyMetrics::graphDist(graph, 99, graph.node_to_ix.at(1))));
}

TEST(UncertaintyMetrics, VarianceDijkstraMatchesDistanceWhenWeightsMatch) {
    GraphState graph;
    for (int node_id = 0; node_id <= 3; ++node_id) {
        addNode(graph, node_id);
    }

    addDirectedEdge(graph, 0, 1, 1.0f, 1.0f);
    addDirectedEdge(graph, 1, 2, 2.0f, 2.0f);
    addDirectedEdge(graph, 2, 3, 3.0f, 3.0f);

    EXPECT_NEAR(UncertaintyMetrics::graphDist(graph, graph.node_to_ix.at(0),
                                              graph.node_to_ix.at(3)),
                UncertaintyMetrics::graphVarianceDist(
                    graph, graph.node_to_ix.at(0), graph.node_to_ix.at(3)),
                1e-4f);
}

TEST(UncertaintyMetrics, VarianceDijkstraPrefersLowerVariancePath) {
    GraphState graph;
    for (int node_id = 0; node_id <= 3; ++node_id) {
        addNode(graph, node_id);
    }

    addDirectedEdge(graph, 0, 1, 2.0f, 10.0f);
    addDirectedEdge(graph, 1, 3, 2.0f, 10.0f);
    addDirectedEdge(graph, 0, 2, 2.0f, 1.0f);
    addDirectedEdge(graph, 2, 3, 2.0f, 1.0f);

    EXPECT_NEAR(UncertaintyMetrics::graphVarianceDist(
                    graph, graph.node_to_ix.at(0), graph.node_to_ix.at(3)),
                2.0f, 1e-4f);
}

TEST(UncertaintyMetrics, ZeroVarianceLoopClosureCollapsesPathVariance) {
    GraphState graph;
    for (int node_id = 0; node_id <= 3; ++node_id) {
        addNode(graph, node_id);
    }

    addDirectedEdge(graph, 0, 1, 1.0f, 1.0f);
    addDirectedEdge(graph, 1, 2, 1.0f, 1.0f);
    addDirectedEdge(graph, 2, 3, 1.0f, 1.0f);
    addDirectedEdge(graph, 0, 3, 5.0f, 0.0f);

    EXPECT_FLOAT_EQ(UncertaintyMetrics::graphVarianceDist(
                        graph, graph.node_to_ix.at(0), graph.node_to_ix.at(3)),
                    0.0f);
}

TEST(UncertaintyMetrics, RotationRiskLambdaUsesPlanarLogDet) {
    Params params;
    params.rotation_risk_enabled = true;
    params.rotation_risk_weight = 0.5f;
    params.rotation_risk_reference_det = 1.0e-6f;
    params.rotation_risk_max_lambda = 10.0f;

    const auto covariance = makeCovariance(0.1, 0.2, 0.3);
    const float lambda = UncertaintyMetrics::rotationRiskLambdaFromCovariance(
        covariance, params);

    const double expected = 0.5 * std::log((0.1 * 0.2 * 0.3) / 1.0e-6);
    EXPECT_NEAR(lambda, expected, 1e-4f);
}

TEST(UncertaintyMetrics, RotationRiskLambdaHandlesSingularWithJitter) {
    Params params;
    params.rotation_risk_enabled = true;
    params.rotation_risk_weight = 1.0f;
    params.rotation_risk_reference_det = 1.0e-30f;
    params.rotation_risk_max_lambda = 10.0f;

    const auto covariance = makeCovariance(0.0, 0.0, 0.0);
    const float lambda = UncertaintyMetrics::rotationRiskLambdaFromCovariance(
        covariance, params);

    EXPECT_TRUE(std::isfinite(lambda));
    EXPECT_GT(lambda, 0.0f);
}

TEST(UncertaintyMetrics, RotationRiskLambdaInvalidCovarianceReturnsZero) {
    Params params;
    params.rotation_risk_enabled = true;

    auto nan_covariance = makeCovariance(0.1, 0.1, 0.1);
    nan_covariance[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FLOAT_EQ(UncertaintyMetrics::rotationRiskLambdaFromCovariance(
                        nan_covariance, params),
                    0.0f);

    const auto negative_covariance = makeCovariance(0.1, -0.1, 0.1);
    EXPECT_FLOAT_EQ(UncertaintyMetrics::rotationRiskLambdaFromCovariance(
                        negative_covariance, params),
                    0.0f);
}

TEST(UncertaintyMetrics, RotationRiskLambdaClampsAtMax) {
    Params params;
    params.rotation_risk_enabled = true;
    params.rotation_risk_weight = 10.0f;
    params.rotation_risk_reference_det = 1.0e-12f;
    params.rotation_risk_max_lambda = 1.25f;

    const auto covariance = makeCovariance(1.0, 1.0, 1.0);
    EXPECT_NEAR(UncertaintyMetrics::rotationRiskLambdaFromCovariance(covariance,
                                                                     params),
                1.25f, 1e-6f);
}

}  // namespace alc_planner
