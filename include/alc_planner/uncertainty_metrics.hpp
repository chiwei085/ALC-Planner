#pragma once

#include <unordered_map>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class UncertaintyMetrics
{
public:
    static std::unordered_map<int, float> dijkstraAll(const GraphState& graph,
                                                      int from_id);

    static float graphDist(const GraphState& graph, int from_id, int to_id);
};

}  // namespace alc_planner
