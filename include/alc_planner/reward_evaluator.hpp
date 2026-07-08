#pragma once

#include "alc_planner/types.hpp"

namespace alc_planner
{

class RewardEvaluator
{
public:
    explicit RewardEvaluator(Params params);

    void fillRewardUB(ALCCandidate& candidate,
                      const SaliencyState& saliency_state) const;

    void fillReward(ALCCandidate& candidate,
                    const SaliencyState& saliency_state) const;

    [[nodiscard]] float computeRotationRisk(ALCCandidate& candidate,
                                            float lambda) const;

private:
    Params params_;
};

}  // namespace alc_planner
