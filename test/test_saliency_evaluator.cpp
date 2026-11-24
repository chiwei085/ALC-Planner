#include <gtest/gtest.h>

#include <cmath>
#include <numeric>

#include "alc_planner/saliency_evaluator.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{
namespace
{

int addKeyframe(GraphState& graph, SaliencyState& saliency_state,
                const int node_id, std::vector<int32_t> word_ids) {
    const int ix = static_cast<int>(graph.keyframes.size());
    Keyframe keyframe;
    keyframe.node_id = node_id;
    keyframe.word_ids = std::move(word_ids);
    graph.node_to_ix[node_id] = ix;
    graph.ix_to_node.push_back(node_id);
    graph.keyframes.push_back(std::move(keyframe));
    graph.adj.push_back({});
    saliency_state.keyframes.push_back({});
    return ix;
}

}  // namespace

TEST(SaliencyEvaluator, SLNormalizesCorrectly) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    std::vector<int32_t> words_100(100);
    std::vector<int32_t> words_50(50);
    std::iota(words_100.begin(), words_100.end(), 1);
    std::iota(words_50.begin(), words_50.end(), 101);

    const int ix1 = addKeyframe(graph, saliency_state, 1, std::move(words_100));
    const int ix2 = addKeyframe(graph, saliency_state, 2, std::move(words_50));

    SaliencyEvaluator evaluator(params);
    evaluator.observeWordsRecognized(100);
    evaluator.update(graph, saliency_state);

    EXPECT_NEAR(
        saliency_state.keyframes[static_cast<std::size_t>(ix1)].saliency_local,
        1.0f, 1e-4f);
    EXPECT_NEAR(
        saliency_state.keyframes[static_cast<std::size_t>(ix2)].saliency_local,
        0.5f, 1e-4f);
    EXPECT_GT(
        saliency_state.keyframes[static_cast<std::size_t>(ix1)].plc_intrinsic,
        saliency_state.keyframes[static_cast<std::size_t>(ix2)].plc_intrinsic);
}

TEST(SaliencyEvaluator, SGRarityOrder) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix1 = addKeyframe(graph, saliency_state, 1, {1, 2});
    const int ix2 = addKeyframe(graph, saliency_state, 2, {3, 4});
    addKeyframe(graph, saliency_state, 3, {3, 4, 5, 6});

    SaliencyEvaluator evaluator(params);
    evaluator.observeWordsRecognized(4);
    evaluator.update(graph, saliency_state);

    EXPECT_GT(
        saliency_state.keyframes[static_cast<std::size_t>(ix1)].saliency_global,
        saliency_state.keyframes[static_cast<std::size_t>(ix2)]
            .saliency_global);
}

TEST(SaliencyEvaluator, EmptyWordIdsNoCrash) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addKeyframe(graph, saliency_state, 1, {});
    graph.robot_ix = ix;

    SaliencyEvaluator evaluator(params);
    evaluator.update(graph, saliency_state);

    EXPECT_FLOAT_EQ(
        saliency_state.keyframes[static_cast<std::size_t>(ix)].saliency_local,
        0.0f);
    EXPECT_FLOAT_EQ(
        saliency_state.keyframes[static_cast<std::size_t>(ix)].saliency_global,
        0.0f);
    EXPECT_FLOAT_EQ(
        saliency_state.keyframes[static_cast<std::size_t>(ix)].plc_intrinsic,
        0.0f);
    EXPECT_FLOAT_EQ(evaluator.latestSL(), 0.0f);
}

TEST(SaliencyEvaluator, SingleNodeSGIsZero) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addKeyframe(graph, saliency_state, 1, {10, 20, 30});

    SaliencyEvaluator evaluator(params);
    evaluator.update(graph, saliency_state);

    EXPECT_FLOAT_EQ(
        saliency_state.keyframes[static_cast<std::size_t>(ix)].saliency_global,
        0.0f);
}

TEST(SaliencyEvaluator, LoopClosureCalibrationDefaultsToOne) {
    Params params;
    SaliencyEvaluator evaluator(params);

    EXPECT_FLOAT_EQ(evaluator.loopClosureCalibration(), 1.0f);
}

TEST(SaliencyEvaluator, LoopClosureCalibrationIsClampedLow) {
    Params params;
    SaliencyEvaluator evaluator(params);
    for (int i = 0; i < 10; ++i) {
        evaluator.observeLoopClosureAttempt(false);
    }

    EXPECT_FLOAT_EQ(evaluator.loopClosureCalibration(), 0.5f);
}

TEST(SaliencyEvaluator, LoopClosureCalibrationIsClampedHigh) {
    Params params;
    SaliencyEvaluator evaluator(params);
    for (int i = 0; i < 10; ++i) {
        evaluator.observeLoopClosureAttempt(true);
    }

    EXPECT_FLOAT_EQ(evaluator.loopClosureCalibration(), 2.0f);
}

TEST(SaliencyEvaluator, LoopClosureCalibrationFollowsSuccessRate) {
    Params params;
    SaliencyEvaluator evaluator(params);
    for (const bool success : {true, true, true, false}) {
        evaluator.observeLoopClosureAttempt(success);
    }

    // Success rate 3/4 over the prior rate 1/2.
    EXPECT_FLOAT_EQ(evaluator.loopClosureCalibration(), 1.5f);
}

TEST(SaliencyEvaluator, UpdateStoresLoopClosureCalibrationSeparately) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    const int ix = addKeyframe(graph, saliency_state, 1, {10, 20, 30, 40});
    // A second node without shared words makes S_G, and so the PLC, nonzero.
    addKeyframe(graph, saliency_state, 2, {50, 60, 70, 80});

    SaliencyEvaluator baseline(params);
    baseline.observeWordsRecognized(4);
    baseline.update(graph, saliency_state);
    const float base_plc =
        saliency_state.keyframes[static_cast<std::size_t>(ix)].plc_intrinsic;
    // S_L = S_G = 1.
    EXPECT_NEAR(base_plc, std::tanh(params.cv_L) * std::tanh(params.cv_G),
                1e-4f);
    EXPECT_FLOAT_EQ(saliency_state.plc_calibration, 1.0f);

    SaliencyEvaluator calibrated(params);
    for (int i = 0; i < 10; ++i) {
        calibrated.observeLoopClosureAttempt(true);
    }
    calibrated.observeWordsRecognized(4);
    calibrated.update(graph, saliency_state);

    EXPECT_NEAR(
        saliency_state.keyframes[static_cast<std::size_t>(ix)].plc_intrinsic,
        base_plc, 1e-4f);
    EXPECT_FLOAT_EQ(saliency_state.plc_calibration, 2.0f);
}

TEST(SaliencyEvaluator, UpdateStoresLowerLoopClosureCalibration) {
    Params params;
    GraphState graph;
    SaliencyState saliency_state;
    addKeyframe(graph, saliency_state, 1, {10, 20, 30, 40});

    SaliencyEvaluator evaluator(params);
    for (int i = 0; i < 10; ++i) {
        evaluator.observeLoopClosureAttempt(false);
    }
    evaluator.observeWordsRecognized(4);
    evaluator.update(graph, saliency_state);

    EXPECT_FLOAT_EQ(saliency_state.plc_calibration, 0.5f);
}

}  // namespace alc_planner
