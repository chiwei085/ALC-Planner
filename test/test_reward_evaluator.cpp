#include <gtest/gtest.h>

#include <cmath>

#include "alc_planner/reward_evaluator.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{
namespace
{

int addNode(GraphState& graph, SaliencyState& saliency_state, const int node_id,
            const float plc_intrinsic) {
    const int ix = static_cast<int>(graph.keyframes.size());
    Keyframe keyframe;
    keyframe.node_id = node_id;
    graph.node_to_ix[node_id] = ix;
    graph.ix_to_node.push_back(node_id);
    graph.keyframes.push_back(std::move(keyframe));
    graph.adj.push_back({});

    KeyframeSaliency saliency;
    saliency.plc_intrinsic = plc_intrinsic;
    saliency_state.keyframes.push_back(std::move(saliency));
    return ix;
}

ALCCandidate makeCandidate() {
    ALCCandidate candidate;
    candidate.tau_ix = 0;
    return candidate;
}

float expectedUBProbability(const Params& params, const ALCCandidate& candidate,
                            const SaliencyState& saliency_state) {
    const float l_minus_ub = candidate.graph_dist + candidate.euclidean_dist;
    const float exp_factor =
        std::exp(-(l_minus_ub * l_minus_ub) / (params.cl * params.cl));
    float plc_sum = 0.0f;
    for (const int keyframe_ix : candidate.keyframe_ixs) {
        if (keyframe_ix < 0 ||
            keyframe_ix >= static_cast<int>(saliency_state.keyframes.size())) {
            continue;
        }

        plc_sum +=
            saliency_state.keyframes[static_cast<std::size_t>(keyframe_ix)]
                .plc_intrinsic *
            exp_factor;
    }
    return std::min(1.0f, plc_sum);
}

float expectedExactProbability(const Params& params,
                               const ALCCandidate& candidate,
                               const SaliencyState& saliency_state) {
    const float l_minus = candidate.graph_dist + candidate.map_dist;
    float prob_all_fail = 1.0f;
    for (const int keyframe_ix : candidate.keyframe_ixs) {
        if (keyframe_ix < 0 ||
            keyframe_ix >= static_cast<int>(saliency_state.keyframes.size())) {
            continue;
        }

        const float plc_i =
            saliency_state.keyframes[static_cast<std::size_t>(keyframe_ix)]
                .plc_intrinsic *
            std::exp(-(l_minus * l_minus) / (params.cl * params.cl));
        prob_all_fail *= (1.0f - plc_i);
    }
    return 1.0f - prob_all_fail;
}

}  // namespace

TEST(RewardEvaluator, RewardUBDeltaUubFormula) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 10.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    EXPECT_NEAR(candidate.delta_U_ub, 6.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBDeltaUubNegative) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 2.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    EXPECT_NEAR(candidate.delta_U_ub, -2.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBUsesClusterUnionBound) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix1 = addNode(graph, saliency_state, 1, 0.25f);
    const int ix2 = addNode(graph, saliency_state, 2, 0.45f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix1, ix2};
    candidate.graph_dist = 6.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    const float expected =
        expectedUBProbability(params, candidate, saliency_state);
    EXPECT_NEAR(candidate.P_lc_ub, expected, 1e-4f);
}

TEST(RewardEvaluator, RewardUBZeroSaliency) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.0f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix};
    candidate.graph_dist = 8.0f;
    candidate.euclidean_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    EXPECT_FLOAT_EQ(candidate.P_lc_ub, 0.0f);
    EXPECT_NEAR(candidate.reward_ub, -params.ct * 3.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBExpDecayWithDistance) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.6f);

    ALCCandidate near_candidate = makeCandidate();
    near_candidate.keyframe_ixs = {ix};
    near_candidate.graph_dist = 4.0f;
    near_candidate.euclidean_dist = 3.0f;

    ALCCandidate far_candidate = makeCandidate();
    far_candidate.keyframe_ixs = {ix};
    far_candidate.graph_dist = 10.0f;
    far_candidate.euclidean_dist = 8.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(near_candidate, saliency_state);
    evaluator.fillRewardUB(far_candidate, saliency_state);

    EXPECT_GT(near_candidate.P_lc_ub, far_candidate.P_lc_ub);
}

TEST(RewardEvaluator, RewardUBFormula) {
    Params params;
    params.cv_L = 2.0f;
    params.cv_G = 4.0f;
    params.cl = 5.0f;
    params.ct = 0.2f;

    GraphState graph;
    SaliencyState saliency_state;
    const int ix1 = addNode(graph, saliency_state, 1, 0.35f);
    const int ix2 = addNode(graph, saliency_state, 2, 0.25f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix1, ix2};
    candidate.graph_dist = 7.0f;
    candidate.euclidean_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    const float expected_p =
        expectedUBProbability(params, candidate, saliency_state);
    const float expected_delta_u = 4.0f;
    const float expected_reward = -0.2f * 3.0f + expected_p * expected_delta_u;
    EXPECT_NEAR(candidate.delta_U_ub, expected_delta_u, 1e-4f);
    EXPECT_NEAR(candidate.P_lc_ub, expected_p, 1e-4f);
    EXPECT_NEAR(candidate.reward_ub, expected_reward, 1e-4f);
}

TEST(RewardEvaluator, RewardUBIsValidUpperBound) {
    Params params;
    params.cl = 20.0f;

    GraphState graph;
    SaliencyState saliency_state;
    ALCCandidate candidate = makeCandidate();
    for (int node_id = 1; node_id <= 5; ++node_id) {
        candidate.keyframe_ixs.push_back(
            addNode(graph, saliency_state, node_id, 0.8f));
    }
    candidate.graph_dist = 6.0f;
    candidate.euclidean_dist = 3.0f;
    candidate.map_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);
    evaluator.fillReward(candidate, saliency_state);

    EXPECT_GE(candidate.P_lc_ub + 1e-6f, candidate.P_lc);
    EXPECT_GE(candidate.reward_ub + 1e-6f, candidate.reward);
}

TEST(RewardEvaluator, FillRewardDeltaUFormula) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 9.0f;
    candidate.map_dist = 6.5f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    EXPECT_NEAR(candidate.delta_U, 2.5f, 1e-4f);
}

TEST(RewardEvaluator, FillRewardPLCSingleKeyframe) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.7f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix};
    candidate.graph_dist = 4.0f;
    candidate.map_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    const float expected =
        expectedExactProbability(params, candidate, saliency_state);
    EXPECT_NEAR(candidate.P_lc, expected, 1e-4f);
}

TEST(RewardEvaluator, FillRewardPLCTwoKeyframes) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix1 = addNode(graph, saliency_state, 1, 0.4f);
    const int ix2 = addNode(graph, saliency_state, 2, 0.6f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix1, ix2};
    candidate.graph_dist = 5.0f;
    candidate.map_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    const float expected =
        expectedExactProbability(params, candidate, saliency_state);
    EXPECT_NEAR(candidate.P_lc, expected, 1e-4f);
}

TEST(RewardEvaluator, FillRewardFormula) {
    Params params;
    params.cl = 8.0f;
    params.ct = 0.25f;

    GraphState graph;
    SaliencyState saliency_state;
    const int ix1 = addNode(graph, saliency_state, 1, 0.5f);
    const int ix2 = addNode(graph, saliency_state, 2, 0.3f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix1, ix2};
    candidate.graph_dist = 10.0f;
    candidate.map_dist = 6.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    const float expected_p =
        expectedExactProbability(params, candidate, saliency_state);
    const float expected_reward =
        -params.ct * candidate.map_dist + expected_p * (10.0f - 6.0f);
    EXPECT_NEAR(candidate.P_lc, expected_p, 1e-4f);
    EXPECT_NEAR(candidate.reward, expected_reward, 1e-4f);
}

TEST(RewardEvaluator, RewardUBUsesVarianceDistanceWhenEnabled) {
    Params params;
    params.use_variance_uncertainty = true;

    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.4f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix};
    candidate.graph_dist = 10.0f;
    candidate.graph_dist_var = 1.5f;
    candidate.euclidean_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, saliency_state);

    EXPECT_NEAR(candidate.delta_U_ub, 1.5f, 1e-4f);
}

TEST(RewardEvaluator, FillRewardUsesVarianceDistanceWhenEnabled) {
    Params params;
    params.use_variance_uncertainty = true;

    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.5f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix};
    candidate.graph_dist = 10.0f;
    candidate.graph_dist_var = 2.25f;
    candidate.map_dist = 6.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    EXPECT_NEAR(candidate.delta_U, 2.25f, 1e-4f);
}

TEST(RewardEvaluator, RewardUsesCalibratedPLCWithoutMutatingIntrinsic) {
    Params params;
    GraphState baseline_graph;
    SaliencyState baseline_saliency_state;
    GraphState calibrated_graph;
    SaliencyState calibrated_saliency_state;
    calibrated_saliency_state.plc_calibration = 2.0f;
    const int baseline_ix =
        addNode(baseline_graph, baseline_saliency_state, 1, 0.7f);
    const int calibrated_ix =
        addNode(calibrated_graph, calibrated_saliency_state, 1, 0.7f);

    ALCCandidate baseline_candidate = makeCandidate();
    baseline_candidate.keyframe_ixs = {baseline_ix};
    baseline_candidate.graph_dist = 4.0f;
    baseline_candidate.map_dist = 3.0f;

    ALCCandidate calibrated_candidate = makeCandidate();
    calibrated_candidate.keyframe_ixs = {calibrated_ix};
    calibrated_candidate.graph_dist = 4.0f;
    calibrated_candidate.map_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(baseline_candidate, baseline_saliency_state);
    evaluator.fillReward(calibrated_candidate, calibrated_saliency_state);

    EXPECT_FLOAT_EQ(calibrated_saliency_state
                        .keyframes[static_cast<std::size_t>(calibrated_ix)]
                        .plc_intrinsic,
                    0.7f);
    EXPECT_GE(calibrated_candidate.P_lc, baseline_candidate.P_lc);
    EXPECT_GE(calibrated_candidate.reward, baseline_candidate.reward);
    EXPECT_LE(calibrated_candidate.P_lc, 1.0f);
}

TEST(RewardEvaluator, RotationRiskDisabledLeavesRewardUnchanged) {
    Params params;
    params.rotation_risk_enabled = false;
    params.cl = 8.0f;

    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addNode(graph, saliency_state, 1, 0.8f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ixs = {ix};
    candidate.graph_dist = 8.0f;
    candidate.map_dist = 4.0f;
    candidate.approach_yaw_delta = 3.1415926f;
    candidate.pose_uncertainty_lambda = 5.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    EXPECT_FLOAT_EQ(candidate.rotation_risk, 0.0f);
    EXPECT_NEAR(candidate.reward, candidate.reward_before_rotation_risk, 1e-4f);
}

TEST(RewardEvaluator, RotationRiskZeroLambdaLeavesRewardUnchanged) {
    Params params;
    params.rotation_risk_enabled = true;

    GraphState graph;
    SaliencyState saliency_state;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 6.0f;
    candidate.map_dist = 3.0f;
    candidate.approach_yaw_delta = 1.0f;
    candidate.pose_uncertainty_lambda = 0.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, saliency_state);

    EXPECT_FLOAT_EQ(candidate.rotation_risk, 0.0f);
    EXPECT_NEAR(candidate.reward, candidate.reward_before_rotation_risk, 1e-4f);
}

TEST(RewardEvaluator, HigherYawDeltaLowersRewardMore) {
    Params params;
    params.rotation_risk_enabled = true;
    params.rotation_risk_max_yaw_rad = 2.0f;

    GraphState graph;
    SaliencyState saliency_state;

    ALCCandidate low_turn = makeCandidate();
    low_turn.graph_dist = 6.0f;
    low_turn.map_dist = 3.0f;
    low_turn.approach_yaw_delta = 0.5f;
    low_turn.pose_uncertainty_lambda = 2.0f;

    ALCCandidate high_turn = low_turn;
    high_turn.approach_yaw_delta = 1.5f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(low_turn, saliency_state);
    evaluator.fillReward(high_turn, saliency_state);

    EXPECT_LT(low_turn.rotation_risk, high_turn.rotation_risk);
    EXPECT_GT(low_turn.reward, high_turn.reward);
}

TEST(RewardEvaluator, StrongUtilityCanStillBeatLowerRiskCandidate) {
    Params params;
    params.rotation_risk_enabled = true;
    params.rotation_risk_max_yaw_rad = 3.1415926f;
    params.cl = 20.0f;

    GraphState graph;
    SaliencyState saliency_state;
    const int weak_ix = addNode(graph, saliency_state, 1, 0.15f);
    const int strong_ix = addNode(graph, saliency_state, 2, 0.9f);

    ALCCandidate low_risk = makeCandidate();
    low_risk.keyframe_ixs = {weak_ix};
    low_risk.graph_dist = 5.0f;
    low_risk.map_dist = 3.0f;
    low_risk.approach_yaw_delta = 0.1f;
    low_risk.pose_uncertainty_lambda = 0.5f;

    ALCCandidate high_risk_strong_utility = makeCandidate();
    high_risk_strong_utility.keyframe_ixs = {strong_ix};
    high_risk_strong_utility.graph_dist = 12.0f;
    high_risk_strong_utility.map_dist = 3.0f;
    high_risk_strong_utility.approach_yaw_delta = 2.5f;
    high_risk_strong_utility.pose_uncertainty_lambda = 0.5f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(low_risk, saliency_state);
    evaluator.fillReward(high_risk_strong_utility, saliency_state);

    EXPECT_GT(high_risk_strong_utility.rotation_risk, low_risk.rotation_risk);
    EXPECT_GT(high_risk_strong_utility.reward, low_risk.reward);
}

}  // namespace alc_planner
