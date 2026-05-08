#pragma once

#include <unordered_map>
#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class CandidateBuilder
{
public:
    explicit CandidateBuilder(const Params& params);

    std::vector<ALCCandidate> build(const GraphState& graph) const;

private:
    std::vector<int> filterKeyframes(
        const GraphState& graph,
        const std::unordered_map<int, float>& dist_map) const;

    std::vector<std::vector<int>> dbscan(const GraphState& graph,
                                         const std::vector<int>& ids) const;

    int selectRepresentative(const GraphState& graph,
                             const std::vector<int>& cluster_ids,
                             const Pose6f& robot_pose) const;

    const Params& params_;
};

}  // namespace alc_planner
