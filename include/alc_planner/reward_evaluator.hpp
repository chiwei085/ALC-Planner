#pragma once

#include "alc_planner/types.hpp"

namespace alc_planner
{

class RewardEvaluator
{
public:
    explicit RewardEvaluator(const Params& params);

    void fillRewardUB(ALCCandidate& candidate, const GraphState& graph) const;

    void fillReward(ALCCandidate& candidate, const GraphState& graph) const;

private:
    const Params& params_;
};

}  // namespace alc_planner
