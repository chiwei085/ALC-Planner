#include <gtest/gtest.h>

#include <cmath>

#include "alc_planner/reward_evaluator.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{
namespace
{

void addNode(GraphState& graph, const int id, const float sl, const float sg,
             const float plc_intrinsic) {
    Keyframe keyframe;
    keyframe.node_id = id;
    keyframe.saliency_local = sl;
    keyframe.saliency_global = sg;
    keyframe.plc_intrinsic = plc_intrinsic;
    graph.keyframes.emplace(id, keyframe);
}

ALCCandidate makeCandidate() {
    ALCCandidate candidate;
    candidate.tau_id = 1;
    return candidate;
}

float expectedUBProbability(const Params& params, const ALCCandidate& candidate,
                            const GraphState& graph) {
    const float l_minus_ub = candidate.graph_dist + candidate.euclidean_dist;
    const float exp_factor =
        std::exp(-(l_minus_ub * l_minus_ub) / (params.cl * params.cl));
    float plc_sum = 0.0f;
    for (const int node_id : candidate.keyframe_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        plc_sum += it->second.plc_intrinsic * exp_factor;
    }
    return std::min(1.0f, plc_sum);
}

float expectedExactProbability(const Params& params,
                               const ALCCandidate& candidate,
                               const GraphState& graph) {
    const float l_minus = candidate.graph_dist + candidate.map_dist;
    float prob_all_fail = 1.0f;
    for (const int node_id : candidate.keyframe_ids) {
        const auto it = graph.keyframes.find(node_id);
        if (it == graph.keyframes.end()) {
            continue;
        }

        const float plc_i =
            it->second.plc_intrinsic *
            std::exp(-(l_minus * l_minus) / (params.cl * params.cl));
        prob_all_fail *= (1.0f - plc_i);
    }
    return 1.0f - prob_all_fail;
}

}  // namespace

TEST(RewardEvaluator, RewardUBDeltaUubFormula) {
    Params params;
    GraphState graph;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 10.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);

    EXPECT_NEAR(candidate.delta_U_ub, 6.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBDeltaUubNegative) {
    Params params;
    GraphState graph;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 2.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);

    EXPECT_NEAR(candidate.delta_U_ub, -2.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBUsesClusterUnionBound) {
    Params params;
    GraphState graph;
    addNode(graph, 1, 0.2f, 0.4f, 0.25f);
    addNode(graph, 2, 0.8f, 0.5f, 0.45f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1, 2};
    candidate.graph_dist = 6.0f;
    candidate.euclidean_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);

    const float expected = expectedUBProbability(params, candidate, graph);
    EXPECT_NEAR(candidate.P_lc_ub, expected, 1e-4f);
}

TEST(RewardEvaluator, RewardUBZeroSaliency) {
    Params params;
    GraphState graph;
    addNode(graph, 1, 0.0f, 0.7f, 0.0f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1};
    candidate.graph_dist = 8.0f;
    candidate.euclidean_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);

    EXPECT_FLOAT_EQ(candidate.P_lc_ub, 0.0f);
    EXPECT_NEAR(candidate.reward_ub, -params.ct * 3.0f, 1e-4f);
}

TEST(RewardEvaluator, RewardUBExpDecayWithDistance) {
    Params params;
    GraphState graph;
    addNode(graph, 1, 0.8f, 0.8f, 0.6f);

    ALCCandidate near_candidate = makeCandidate();
    near_candidate.keyframe_ids = {1};
    near_candidate.graph_dist = 4.0f;
    near_candidate.euclidean_dist = 3.0f;

    ALCCandidate far_candidate = makeCandidate();
    far_candidate.keyframe_ids = {1};
    far_candidate.graph_dist = 10.0f;
    far_candidate.euclidean_dist = 8.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(near_candidate, graph);
    evaluator.fillRewardUB(far_candidate, graph);

    EXPECT_GT(near_candidate.P_lc_ub, far_candidate.P_lc_ub);
}

TEST(RewardEvaluator, RewardUBFormula) {
    Params params;
    params.cv_L = 2.0f;
    params.cv_G = 4.0f;
    params.cl = 5.0f;
    params.ct = 0.2f;

    GraphState graph;
    addNode(graph, 1, 0.6f, 0.4f, 0.35f);
    addNode(graph, 2, 0.3f, 0.9f, 0.25f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1, 2};
    candidate.graph_dist = 7.0f;
    candidate.euclidean_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);

    const float expected_p = expectedUBProbability(params, candidate, graph);
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
    for (int node_id = 1; node_id <= 5; ++node_id) {
        addNode(graph, node_id, 0.0f, 0.0f, 0.8f);
    }

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1, 2, 3, 4, 5};
    candidate.graph_dist = 6.0f;
    candidate.euclidean_dist = 3.0f;
    candidate.map_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillRewardUB(candidate, graph);
    evaluator.fillReward(candidate, graph);

    EXPECT_GE(candidate.P_lc_ub + 1e-6f, candidate.P_lc);
    EXPECT_GE(candidate.reward_ub + 1e-6f, candidate.reward);
}

TEST(RewardEvaluator, FillRewardDeltaUFormula) {
    Params params;
    GraphState graph;
    ALCCandidate candidate = makeCandidate();
    candidate.graph_dist = 9.0f;
    candidate.map_dist = 6.5f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, graph);

    EXPECT_NEAR(candidate.delta_U, 2.5f, 1e-4f);
}

TEST(RewardEvaluator, FillRewardPLCSingleKeyframe) {
    Params params;
    GraphState graph;
    addNode(graph, 1, 0.0f, 0.0f, 0.7f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1};
    candidate.graph_dist = 4.0f;
    candidate.map_dist = 3.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, graph);

    const float expected = expectedExactProbability(params, candidate, graph);
    EXPECT_NEAR(candidate.P_lc, expected, 1e-4f);
}

TEST(RewardEvaluator, FillRewardPLCTwoKeyframes) {
    Params params;
    GraphState graph;
    addNode(graph, 1, 0.0f, 0.0f, 0.4f);
    addNode(graph, 2, 0.0f, 0.0f, 0.6f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1, 2};
    candidate.graph_dist = 5.0f;
    candidate.map_dist = 4.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, graph);

    const float expected = expectedExactProbability(params, candidate, graph);
    EXPECT_NEAR(candidate.P_lc, expected, 1e-4f);
}

TEST(RewardEvaluator, FillRewardFormula) {
    Params params;
    params.cl = 8.0f;
    params.ct = 0.25f;

    GraphState graph;
    addNode(graph, 1, 0.0f, 0.0f, 0.5f);
    addNode(graph, 2, 0.0f, 0.0f, 0.3f);

    ALCCandidate candidate = makeCandidate();
    candidate.keyframe_ids = {1, 2};
    candidate.graph_dist = 10.0f;
    candidate.map_dist = 6.0f;

    RewardEvaluator evaluator(params);
    evaluator.fillReward(candidate, graph);

    const float expected_p = expectedExactProbability(params, candidate, graph);
    const float expected_reward =
        -params.ct * candidate.map_dist + expected_p * (10.0f - 6.0f);
    EXPECT_NEAR(candidate.P_lc, expected_p, 1e-4f);
    EXPECT_NEAR(candidate.reward, expected_reward, 1e-4f);
}

}  // namespace alc_planner
