#include "alc_planner/uncertainty_metrics.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace alc_planner {

std::unordered_map<int, float> UncertaintyMetrics::dijkstraAll(const GraphState& graph,
                                                               const int from_id) {
  std::unordered_map<int, float> dist;
  if (graph.keyframes.find(from_id) == graph.keyframes.end()) {
    return dist;
  }

  using QueueEntry = std::pair<float, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;

  dist[from_id] = 0.0f;
  pq.push({0.0f, from_id});

  while (!pq.empty()) {
    const auto [curr_dist, node_id] = pq.top();
    pq.pop();

    const auto dist_it = dist.find(node_id);
    if (dist_it == dist.end() || curr_dist > dist_it->second) {
      continue;
    }

    const auto adj_it = graph.adj.find(node_id);
    if (adj_it == graph.adj.end()) {
      continue;
    }

    for (const auto& edge : adj_it->second) {
      const float next_dist = curr_dist + edge.dist;
      const auto next_it = dist.find(edge.to);
      if (next_it == dist.end() || next_dist < next_it->second) {
        dist[edge.to] = next_dist;
        pq.push({next_dist, edge.to});
      }
    }
  }

  return dist;
}

float UncertaintyMetrics::graphDist(const GraphState& graph, const int from_id, const int to_id) {
  const auto dist = dijkstraAll(graph, from_id);
  const auto it = dist.find(to_id);
  if (it == dist.end()) {
    return std::numeric_limits<float>::infinity();
  }
  return it->second;
}

}  // namespace alc_planner
