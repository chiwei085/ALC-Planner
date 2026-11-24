#include "alc_planner/trigger_policy.hpp"

#include <cmath>

namespace alc_planner
{

TriggerPolicy::TriggerPolicy(Params params) : params_(params) {}

double TriggerPolicy::threshold(const double elapsed_seconds,
                                const float coverage_ratio) const {
    const double decayed =
        static_cast<double>(params_.theta_max) *
        std::exp(-static_cast<double>(params_.lambda_decay) * elapsed_seconds);
    return decayed * (1.0 + static_cast<double>(params_.alpha_cov) *
                                (1.0 - coverage_ratio));
}

bool TriggerPolicy::shouldTriggerALC(const float reward,
                                     const double elapsed_seconds,
                                     const float coverage_ratio) const {
    return reward >
           static_cast<float>(threshold(elapsed_seconds, coverage_ratio));
}

}  // namespace alc_planner
