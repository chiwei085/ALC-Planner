#include "alc_planner/slam_graph_planner.hpp"

#include <algorithm>

namespace alc_planner
{

SLAMGraphPlanner::SLAMGraphPlanner(Params params)
    : params_(params), trigger_(std::move(params)) {}

std::optional<ALCCandidate> SLAMGraphPlanner::onEvaluationComplete(
    const std::optional<ALCCandidate>& best, const double elapsed_seconds,
    const float coverage_ratio, const int robot_ix, const float reward_bonus) {
    if (state_ != PlannerState::EVALUATING || !best.has_value()) {
        return std::nullopt;
    }

    if (std::max(0, robot_ix - best->tau_ix) < params_.tau_min_revisit ||
        best->P_lc < params_.plc_min_revisit ||
        best->map_dist < params_.map_dist_min_revisit) {
        return std::nullopt;
    }

    const float effective_reward = best->reward + std::max(0.0f, reward_bonus);
    if (!trigger_.shouldTriggerALC(effective_reward, elapsed_seconds,
                                   coverage_ratio)) {
        return std::nullopt;
    }

    state_ = PlannerState::NAVIGATING_TO_ALC;
    return best;
}

bool SLAMGraphPlanner::onNavigationResult(const bool success) {
    if (state_ != PlannerState::NAVIGATING_TO_ALC) {
        return false;
    }

    if (success) {
        state_ = PlannerState::ROTATING;
        return true;
    }

    state_ = PlannerState::EVALUATING;
    return false;
}

void SLAMGraphPlanner::onRotationComplete() {
    state_ = PlannerState::EVALUATING;
}

PlannerState SLAMGraphPlanner::state() const {
    return state_;
}

}  // namespace alc_planner
