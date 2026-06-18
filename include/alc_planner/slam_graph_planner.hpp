#pragma once

#include <optional>

#include "alc_planner/trigger_policy.hpp"
#include "alc_planner/types.hpp"

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
    explicit SLAMGraphPlanner(Params params);

    [[nodiscard]] std::optional<ALCCandidate> onEvaluationComplete(
        const std::optional<ALCCandidate>& best, double elapsed_seconds,
        float coverage_ratio, int robot_ix, float reward_bonus = 0.0f);

    [[nodiscard]] bool onNavigationResult(bool success);
    void onRotationComplete();

    PlannerState state() const;

private:
    Params params_;
    TriggerPolicy trigger_;
    PlannerState state_ = PlannerState::EVALUATING;
};

}  // namespace alc_planner
