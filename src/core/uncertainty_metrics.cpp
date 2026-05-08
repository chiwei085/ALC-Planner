#include "alc_planner/uncertainty_metrics.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace alc_planner
{

namespace
{

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

}  // namespace alc_planner
