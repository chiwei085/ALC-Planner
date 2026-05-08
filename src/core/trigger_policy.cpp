#include "alc_planner/trigger_policy.hpp"

#include <cmath>

namespace alc_planner
{

TriggerPolicy::TriggerPolicy(Params params) : params_(params) {}

bool TriggerPolicy::shouldTriggerALC(const float reward,
                                     const double elapsed_seconds,
                                     const float coverage_ratio) const {
    double threshold =
        static_cast<double>(params_.theta_max) *
        std::exp(-static_cast<double>(params_.lambda_decay) * elapsed_seconds);
    threshold *=
        1.0 + static_cast<double>(params_.alpha_cov) * (1.0 - coverage_ratio);
    return reward > static_cast<float>(threshold);
}

}  // namespace alc_planner
