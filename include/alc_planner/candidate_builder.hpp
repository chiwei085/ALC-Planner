#pragma once

#include <optional>
#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class CandidateBuilder
{
public:
    explicit CandidateBuilder(Params params);

    std::vector<ALCCandidate> build(const GraphState& graph,
                                    const SaliencyState& saliency_state);

private:
    struct DijkstraKey
    {
        std::uint64_t version = 0;
        int source_ix = -1;

        bool operator==(const DijkstraKey& other) const {
            return version == other.version && source_ix == other.source_ix;
        }
    };

    struct DijkstraResult
    {
        std::vector<float> by_dist;
        std::optional<std::vector<float>> by_variance;
    };

    struct DijkstraCache
    {
        DijkstraKey key;
        DijkstraResult result;
    };

    std::vector<int> filterKeyframes(const GraphState& graph,
                                     const SaliencyState& saliency_state,
                                     const std::vector<float>& dist_map) const;

    std::vector<std::vector<int>> dbscan(const GraphState& graph,
                                         const std::vector<int>& ixs) const;

    int selectRepresentative(const GraphState& graph,
                             const SaliencyState& saliency_state,
                             const std::vector<int>& cluster_ixs,
                             const Pose6f& robot_pose) const;

    DijkstraResult computeDijkstra(const GraphState& graph) const;

    Params params_;
    std::optional<DijkstraCache> dijkstra_cache_;
};

}  // namespace alc_planner
