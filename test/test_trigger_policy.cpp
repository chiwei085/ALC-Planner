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
    candidate.tau_id = 7;
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
        planner.onEvaluationComplete(makeCandidate(0.8f), 0.0, 1.0f);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->tau_id, 7);
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, EvaluateWithoutTriggerReturnsNullopt) {
    Params params;
    params.theta_max = 1.0f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.2f), 0.0, 1.0f);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, EvaluateWithNoBestReturnsNullopt) {
    Params params;
    SLAMGraphPlanner planner(params);

    const auto result = planner.onEvaluationComplete(std::nullopt, 0.0, 1.0f);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::EVALUATING);
}

TEST(SLAMGraphPlanner, NoReentrantNavWhileNavigating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f)
                    .has_value());

    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(planner.state(), PlannerState::NAVIGATING_TO_ALC);
}

TEST(SLAMGraphPlanner, NavSuccessTransitionsToRotating) {
    Params params;
    params.theta_max = 0.1f;
    params.lambda_decay = 0.0f;
    params.alpha_cov = 0.0f;

    SLAMGraphPlanner planner(params);
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f)
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
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f)
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
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f)
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
    ASSERT_TRUE(planner.onEvaluationComplete(makeCandidate(0.5f), 0.0, 1.0f)
                    .has_value());
    ASSERT_TRUE(planner.onNavigationResult(true));
    planner.onRotationComplete();

    const auto result =
        planner.onEvaluationComplete(makeCandidate(0.6f), 0.0, 1.0f);
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
