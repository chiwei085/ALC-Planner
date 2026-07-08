#include "alc_planner/uncertainty_metrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace alc_planner
{

namespace
{

constexpr double kCovarianceJitter = 1.0e-9;

template <typename WeightFn>
std::vector<float> runDijkstra(const GraphState& graph, const int from_ix,
                               WeightFn&& weight_fn) {
    const int node_count = static_cast<int>(graph.keyframes.size());
    std::vector<float> dist(static_cast<std::size_t>(node_count),
                            std::numeric_limits<float>::infinity());
    if (from_ix < 0 || from_ix >= node_count ||
        static_cast<std::size_t>(node_count) != graph.adj.size()) {
        return dist;
    }

    using QueueEntry = std::pair<float, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
        pq;

    dist[static_cast<std::size_t>(from_ix)] = 0.0f;
    pq.push({0.0f, from_ix});

    while (!pq.empty()) {
        const auto [curr_dist, node_ix] = pq.top();
        pq.pop();

        if (curr_dist > dist[static_cast<std::size_t>(node_ix)]) {
            continue;
        }

        for (const auto& edge : graph.adj[static_cast<std::size_t>(node_ix)]) {
            if (edge.to < 0 || edge.to >= node_count) {
                continue;
            }

            const float next_dist = curr_dist + weight_fn(edge);
            if (next_dist < dist[static_cast<std::size_t>(edge.to)]) {
                dist[static_cast<std::size_t>(edge.to)] = next_dist;
                pq.push({next_dist, edge.to});
            }
        }
    }

    return dist;
}

float lookupDist(const std::vector<float>& dist, const int to_ix) {
    if (to_ix < 0 || to_ix >= static_cast<int>(dist.size())) {
        return std::numeric_limits<float>::infinity();
    }
    return dist[static_cast<std::size_t>(to_ix)];
}

double determinant3x3(const double m00, const double m01, const double m02,
                      const double m10, const double m11, const double m12,
                      const double m20, const double m21, const double m22) {
    return m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) +
           m02 * (m10 * m21 - m11 * m20);
}

}  // namespace

std::vector<float> UncertaintyMetrics::dijkstraAll(const GraphState& graph,
                                                   const int from_ix) {
    return runDijkstra(graph, from_ix,
                       [](const GraphEdge& edge) { return edge.dist; });
}

std::vector<float> UncertaintyMetrics::dijkstraVarianceAll(
    const GraphState& graph, const int from_ix) {
    return runDijkstra(graph, from_ix,
                       [](const GraphEdge& edge) { return edge.variance; });
}

float UncertaintyMetrics::graphDist(const GraphState& graph, const int from_ix,
                                    const int to_ix) {
    return lookupDist(dijkstraAll(graph, from_ix), to_ix);
}

float UncertaintyMetrics::graphVarianceDist(const GraphState& graph,
                                            const int from_ix,
                                            const int to_ix) {
    return lookupDist(dijkstraVarianceAll(graph, from_ix), to_ix);
}

float UncertaintyMetrics::rotationRiskLambdaFromCovariance(
    const std::array<double, 36>& covariance, const Params& params) {
    if (!params.rotation_risk_enabled || params.rotation_risk_weight <= 0.0f ||
        params.rotation_risk_reference_det <= 0.0f ||
        params.rotation_risk_max_lambda <= 0.0f) {
        return 0.0f;
    }

    constexpr int kPlanarIndices[3] = {0, 1, 5};
    double planar[3][3]{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int src_row = kPlanarIndices[row];
            const int src_col = kPlanarIndices[col];
            const double forward =
                covariance[static_cast<std::size_t>(src_row * 6 + src_col)];
            const double reverse =
                covariance[static_cast<std::size_t>(src_col * 6 + src_row)];
            if (!std::isfinite(forward) || !std::isfinite(reverse)) {
                return 0.0f;
            }
            planar[row][col] = 0.5 * (forward + reverse);
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (planar[i][i] < 0.0) {
            return 0.0f;
        }
        planar[i][i] += kCovarianceJitter;
    }

    const double det = determinant3x3(planar[0][0], planar[0][1], planar[0][2],
                                      planar[1][0], planar[1][1], planar[1][2],
                                      planar[2][0], planar[2][1], planar[2][2]);
    if (!std::isfinite(det) || det <= 0.0) {
        return 0.0f;
    }

    const double ratio =
        det / static_cast<double>(params.rotation_risk_reference_det);
    const double raw_lambda = static_cast<double>(params.rotation_risk_weight) *
                              std::max(0.0, std::log(ratio));
    if (!std::isfinite(raw_lambda) || raw_lambda <= 0.0) {
        return 0.0f;
    }

    return static_cast<float>(std::min(
        raw_lambda, static_cast<double>(params.rotation_risk_max_lambda)));
}

}  // namespace alc_planner
