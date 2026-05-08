#include "alc_planner/saliency_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace alc_planner
{

SaliencyEvaluator::SaliencyEvaluator(const Params& params) : params_(params) {}

void SaliencyEvaluator::update(GraphState& graph) {
    rebuildWordFrequency(graph);

    const int total_nodes = static_cast<int>(graph.keyframes.size());
    for (auto& [node_id, keyframe] : graph.keyframes) {
        (void)node_id;
        keyframe.saliency_local =
            normalizeSL(static_cast<int>(keyframe.word_ids.size()));
        keyframe.saliency_global = computeSG(keyframe.word_ids, total_nodes);
        keyframe.plc_intrinsic =
            std::tanh(params_.cv_L * keyframe.saliency_local) *
            std::tanh(params_.cv_G * keyframe.saliency_global);
    }

    const auto robot_it = graph.keyframes.find(graph.robot_node_id);
    if (robot_it != graph.keyframes.end()) {
        latest_sl_ = robot_it->second.saliency_local;
    }
}

void SaliencyEvaluator::observeWordsRecognized(const int count) {
    if (count > max_words_recognized_) {
        max_words_recognized_ = count;
    }
}

float SaliencyEvaluator::normalizeSL(const int word_count) const {
    if (word_count <= 0 || max_words_recognized_ <= 0) {
        return 0.0f;
    }

    const float normalized = static_cast<float>(word_count) /
                             static_cast<float>(max_words_recognized_);
    return std::clamp(normalized, 0.0f, 1.0f);
}

float SaliencyEvaluator::computeSG(const std::vector<int32_t>& word_ids,
                                   const int total_nodes) const {
    if (word_ids.empty() || total_nodes <= 1) {
        return 0.0f;
    }

    double sum_inv_freq = 0.0;
    int counted = 0;
    std::unordered_set<int32_t> unique_words;
    for (const int32_t word_id : word_ids) {
        if (!unique_words.insert(word_id).second) {
            continue;
        }

        const auto it = word_node_count_.find(word_id);
        if (it == word_node_count_.end() || it->second <= 0) {
            continue;
        }

        const double freq =
            static_cast<double>(it->second) / static_cast<double>(total_nodes);
        if (freq <= 0.0) {
            continue;
        }

        sum_inv_freq += 1.0 / freq;
        ++counted;
    }

    if (counted == 0) {
        return 0.0f;
    }

    const double raw_sg = sum_inv_freq / static_cast<double>(counted);
    const double denom = std::log(static_cast<double>(total_nodes));
    if (raw_sg <= 1.0 || denom <= 0.0) {
        return 0.0f;
    }

    const double normalized = std::log(raw_sg) / denom;
    return std::clamp(static_cast<float>(normalized), 0.0f, 1.0f);
}

void SaliencyEvaluator::rebuildWordFrequency(const GraphState& graph) {
    word_node_count_.clear();

    for (const auto& [node_id, keyframe] : graph.keyframes) {
        (void)node_id;
        std::unordered_set<int32_t> unique_words(keyframe.word_ids.begin(),
                                                 keyframe.word_ids.end());
        for (const int32_t word_id : unique_words) {
            ++word_node_count_[word_id];
        }
    }
}

}  // namespace alc_planner
