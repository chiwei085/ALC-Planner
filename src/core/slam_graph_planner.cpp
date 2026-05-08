#include "alc_planner/slam_graph_planner.hpp"

namespace alc_planner
{

SLAMGraphPlanner::SLAMGraphPlanner(Params params)
    : trigger_(std::move(params)) {}

std::optional<ALCCandidate> SLAMGraphPlanner::onEvaluationComplete(
    const std::optional<ALCCandidate>& best, const double elapsed_seconds,
    const float coverage_ratio) {
    if (state_ != PlannerState::EVALUATING || !best.has_value()) {
        return std::nullopt;
    }

    if (!trigger_.shouldTriggerALC(best->reward, elapsed_seconds,
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
