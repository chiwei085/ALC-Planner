#pragma once

#include <unordered_map>
#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class SaliencyEvaluator
{
public:
    explicit SaliencyEvaluator(const Params& params);

    void update(GraphState& graph);
    void observeWordsRecognized(int count);
    float latestSL() const { return latest_sl_; }

private:
    const Params& params_;
    int max_words_recognized_ = 1;
    float latest_sl_ = 0.0f;
    std::unordered_map<int32_t, int> word_node_count_;

    float normalizeSL(int word_count) const;
    float computeSG(const std::vector<int32_t>& word_ids,
                    int total_nodes) const;
    void rebuildWordFrequency(const GraphState& graph);
};

}  // namespace alc_planner
