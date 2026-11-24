#pragma once

namespace alc_planner
{

enum class PlannerState
{
    EVALUATING,
    NAVIGATING_TO_ALC,
    ROTATING
};

class SLAMGraphPlanner
{
public:
    // False when an attempt is already under way.
    [[nodiscard]] bool startNavigation();
    // True when the target was reached and a rotation may follow.
    [[nodiscard]] bool onNavigationResult(bool success);
    void onRotationComplete();

    PlannerState state() const;

private:
    PlannerState state_ = PlannerState::EVALUATING;
};

}  // namespace alc_planner
