#include "alc_planner/slam_graph_planner.hpp"

namespace alc_planner
{

bool SLAMGraphPlanner::startNavigation() {
    if (state_ != PlannerState::EVALUATING) {
        return false;
    }
    state_ = PlannerState::NAVIGATING_TO_ALC;
    return true;
}

bool SLAMGraphPlanner::onNavigationResult(const bool success) {
    if (state_ != PlannerState::NAVIGATING_TO_ALC) {
        return false;
    }
    state_ = success ? PlannerState::ROTATING : PlannerState::EVALUATING;
    return success;
}

void SLAMGraphPlanner::onRotationComplete() {
    state_ = PlannerState::EVALUATING;
}

PlannerState SLAMGraphPlanner::state() const {
    return state_;
}

}  // namespace alc_planner
