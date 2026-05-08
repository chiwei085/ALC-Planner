#pragma once

#include "alc_planner/types.hpp"

namespace alc_planner
{

class TriggerPolicy
{
public:
    explicit TriggerPolicy(Params params);

    [[nodiscard]]
    bool shouldTriggerALC(float reward, double elapsed_seconds,
                          float coverage_ratio) const;

private:
    Params params_;
};

}  // namespace alc_planner
