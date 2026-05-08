#pragma once

#include <unordered_map>
#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class SaliencyEvaluator
{
public:
    explicit SaliencyEvaluator(Params params);

    void update(const GraphState& graph, SaliencyState& saliency_state);
    void observeWordsRecognized(int count);
    void observeLoopClosureAttempt(bool success);
    float loopClosureCalibration() const;
    float latestSL() const { return latest_sl_; }

private:
    Params params_;
    int max_words_recognized_ = 1;
    float latest_sl_ = 0.0f;
    int loop_closure_attempts_ = 0;
    int loop_closure_successes_ = 0;
    std::size_t last_processed_kf_count_ = 0;
    std::unordered_map<int32_t, int> word_node_count_;

    float normalizeSL(int word_count) const;
    float computeSG(const std::vector<int32_t>& word_ids,
                    int total_nodes) const;
    void rebuildWordFrequency(const GraphState& graph);
};

}  // namespace alc_planner
