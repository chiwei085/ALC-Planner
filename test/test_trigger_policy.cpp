#include <gtest/gtest.h>

#include <optional>

#include "alc_planner/slam_graph_planner.hpp"
#include "alc_planner/trigger_policy.hpp"

namespace alc_planner
{
namespace
{

ALCCandidate makeCandidate(const float reward) {
    ALCCandidate candidate;
    candidate.tau_ix = 20;
    candidate.P_lc = 1.0f;
    candidate.map_dist = 5.0f;
    candidate.reward = reward;
    return candidate;
}

}  // namespace

TEST(TriggerPolicy, ThresholdDecaysOverTime) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.1f;
    params.alpha_cov = 0.0f;

    TriggerPolicy policy(params);
    EXPECT_FALSE(policy.shouldTriggerALC(0.6f, 0.0, 1.0f));
    EXPECT_TRUE(policy.shouldTriggerALC(0.6f, 10.0, 1.0f));
}

TEST(TriggerPolicy, CoverageRaisesThreshold) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.5f;

    TriggerPolicy policy(params);
    EXPECT_FALSE(policy.shouldTriggerALC(1.2f, 0.0, 0.0f));
    EXPECT_TRUE(policy.shouldTriggerALC(1.2f, 0.0, 1.0f));
}

TEST(TriggerPolicy, TriggerWhenRewardExceedsThreshold) {
    Params params;
    params.theta_max = 0.5f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    TriggerPolicy policy(params);
    EXPECT_TRUE(policy.shouldTriggerALC(0.6f, 0.0, 1.0f));
}

TEST(TriggerPolicy, NoTriggerWhenRewardBelowThreshold) {
    Params params;
    params.theta_max = 0.5f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    TriggerPolicy policy(params);
    EXPECT_FALSE(policy.shouldTriggerALC(0.4f, 0.0, 1.0f));
}

TEST(TriggerPolicy, ZeroElapsedUsesMaxThreshold) {
    Params params;
    params.theta_max = 2.0f;
    params.lambda_decay = 0.5f;
    params.alpha_cov = 0.25f;

    TriggerPolicy policy(params);
    EXPECT_FALSE(policy.shouldTriggerALC(2.49f, 0.0, 0.0f));
    EXPECT_TRUE(policy.shouldTriggerALC(2.51f, 0.0, 0.0f));
}

TEST(TriggerPolicy, HighCoverageTriggersEasier) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.5f;

    TriggerPolicy policy(params);
    EXPECT_FALSE(policy.shouldTriggerALC(1.2f, 0.0, 0.2f));
    EXPECT_TRUE(policy.shouldTriggerALC(1.2f, 0.0, 1.0f));
}

TEST(SLAMGraphPlanner, InitialStateIsEvaluating) {
    Params params;
    SLAMGraphPlanner planner(params);
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, EvaluateWithTriggerReturnsCandidate) {
    Params params;
    params.theta_max = 0.5f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.8f), 0.0, 1.0f, 40);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->tau_ix, 20);
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, EvaluateWithoutTriggerReturnsNullopt) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.2f), 0.0, 1.0f, 40);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, RewardBonusCanTriggerDegradedExplorationCandidate) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    const auto without_bonus =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f, 40);
    EXPECT_FALSE(without_bonus.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);

    const auto with_bonus =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f, 40, 0.5f);
    ASSERT_TRUE(with_bonus.has_value());
    EXPECT_EQ(with_bonus->tau_ix, 20);
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, TauFloorSuppressesNearCandidate) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;
    params.tau_min_revisit = 15;
    params.plc_min_revisit = 0.0f;

    SLAMGraphPlanner planner(params);
    auto candidate = makeCandidate(1.0f);
    candidate.tau_ix = 8;
    candidate.P_lc = 0.9f;

    // robot_ix=20: gap = 20 - 8 = 12 < 15, candidate suppressed
    const auto result = planner.onEvaluationComplete(candidate, 0.0, 1.0f, 20);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, PlcFloorSuppressesLowProbabilityCandidate) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;
    params.tau_min_revisit = 0;
    params.plc_min_revisit = 0.05f;

    SLAMGraphPlanner planner(params);
    auto candidate = makeCandidate(1.0f);
    candidate.tau_ix = 72;
    candidate.P_lc = 0.019f;

    const auto result = planner.onEvaluationComplete(candidate, 0.0, 1.0f, 80);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, CandidatePassingFloorsCanStillTrigger) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;
    params.tau_min_revisit = 15;
    params.plc_min_revisit = 0.05f;

    SLAMGraphPlanner planner(params);
    auto candidate = makeCandidate(1.0f);
    candidate.tau_ix = 31;
    candidate.P_lc = 0.31f;

    // robot_ix=50: gap = 50 - 31 = 19 >= 15, candidate passes tau floor
    const auto result = planner.onEvaluationComplete(candidate, 0.0, 1.0f, 50);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->tau_ix, 31);
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, MapDistFloorSuppressesNearbyCandidate) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;
    params.tau_min_revisit = 15;
    params.plc_min_revisit = 0.05f;
    params.map_dist_min_revisit = 1.0f;

    SLAMGraphPlanner planner(params);
    auto candidate = makeCandidate(1.0f);
    candidate.tau_ix = 42;
    candidate.P_lc = 0.7f;
    candidate.map_dist = 0.365f;

    // robot_ix=60: gap = 60 - 42 = 18 >= 15, suppressed by map_dist_min_revisit
    const auto result = planner.onEvaluationComplete(candidate, 0.0, 1.0f, 60);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, EvaluateWithNoBestReturnsNullopt) {
    Params params;
    SLAMGraphPlanner planner(params);

    const auto result =
        planner.onEvaluationComplete(std::nullopt, 0.0, 1.0f, 40);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, NoReentrantNavWhileNavigating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f, 40)
                    .has_value());

    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f, 40);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, NavSuccessTransitionsToRotating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f, 40)
                    .has_value());

    EXPECT_TRUE(planner.onNavigationResult(true));
    EXPECT_EQ(planner.state(), PlannerState::ROTATING);
}

TEST(SLAMGraphPlanner, NavFailTransitionsToEvaluating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f, 40)
                    .has_value());

    EXPECT_FALSE(planner.onNavigationResult(false));
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, NavResultFromWrongStateReturnsFalse) {
    Params params;
    SLAMGraphPlanner planner(params);

    EXPECT_FALSE(planner.onNavigationResult(false));
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, RotationCompleteResetsToEvaluating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f, 40)
                    .has_value());
    ASSERT_TRUE(planner.onNavigationResult(true));

    planner.onRotationComplete();
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, FullCycleCanTriggerAgain) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f, 40)
                    .has_value());
    ASSERT_TRUE(planner.onNavigationResult(true));
    planner.onRotationComplete();

    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f, 40);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, RotationCompleteFromEvaluatingIsSafe) {
    Params params;
    SLAMGraphPlanner planner(params);

    planner.onRotationComplete();

    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

}  // namespace alc_planner
