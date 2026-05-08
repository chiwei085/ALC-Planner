#include "alc_planner/bnb_selector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace alc_planner
{

BNBSelector::BNBSelector(Params params)
    : params_(params), evaluator_(params_) {}

std::optional<ALCCandidate> BNBSelector::select(
    std::vector<ALCCandidate> candidates, const GraphState& graph,
    const nav_msgs::msg::OccupancyGrid& map) const {
    if (candidates.empty() || graph.robot_node_id < 0) {
        return std::nullopt;
    }

    const auto robot_it = graph.keyframes.find(graph.robot_node_id);
    if (robot_it == graph.keyframes.end()) {
        return std::nullopt;
    }

    const Eigen::Vector3f& robot_pos = robot_it->second.pose.position;
    std::sort(candidates.begin(), candidates.end(),
              [](const ALCCandidate& lhs, const ALCCandidate& rhs) {
                  return lhs.reward_ub > rhs.reward_ub;
              });

    std::optional<ALCCandidate> best_candidate;
    for (auto& candidate : candidates) {
        if (best_candidate.has_value() &&
            candidate.reward_ub <= best_candidate->reward) {
            break;
        }

        candidate.map_dist = path_planner_.computeDist(
            robot_pos, candidate.rep_pose.position, map);
        if (!std::isfinite(candidate.map_dist)) {
            continue;
        }

        evaluator_.fillReward(candidate, graph);
        if (!best_candidate.has_value() ||
            candidate.reward > best_candidate->reward) {
            best_candidate = candidate;
        }
    }

    return best_candidate;
}

}  // namespace alc_planner
