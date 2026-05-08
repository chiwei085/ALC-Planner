#include "alc_planner/reward_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace alc_planner
{

namespace
{

float expDistanceDecay(const float l_minus, const float cl) {
    return std::exp(-(l_minus * l_minus) / (cl * cl));
}

}  // namespace

RewardEvaluator::RewardEvaluator(Params params) : params_(params) {}

void RewardEvaluator::fillRewardUB(ALCCandidate& candidate,
                                   const GraphState& graph) const {
    candidate.delta_U_ub = candidate.graph_dist - candidate.euclidean_dist;
    const float l_minus_ub = candidate.graph_dist + candidate.euclidean_dist;
    const float exp_factor = expDistanceDecay(l_minus_ub, params_.cl);

    float plc_sum = 0.0f;
    for (const int node_id : candidate.keyframe_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        plc_sum += it->second.plc_intrinsic * exp_factor;
    }

    candidate.P_lc_ub = std::min(1.0f, plc_sum);
    candidate.reward_ub = -params_.ct * candidate.euclidean_dist +
                          candidate.P_lc_ub * candidate.delta_U_ub;
}

void RewardEvaluator::fillReward(ALCCandidate& candidate,
                                 const GraphState& graph) const {
    candidate.delta_U = candidate.graph_dist - candidate.map_dist;
    const float l_minus = candidate.graph_dist + candidate.map_dist;

    float prob_all_fail = 1.0f;
    for (const int node_id : candidate.keyframe_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        const float plc_i =
            it->second.plc_intrinsic * expDistanceDecay(l_minus, params_.cl);
        prob_all_fail *= (1.0f - plc_i);
    }

    candidate.P_lc = 1.0f - prob_all_fail;
    candidate.reward =
        -params_.ct * candidate.map_dist + candidate.P_lc * candidate.delta_U;
}

}  // namespace alc_planner
