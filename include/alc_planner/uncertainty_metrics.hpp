#pragma once

#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

class UncertaintyMetrics
{
public:
    static std::vector<float> dijkstraAll(const GraphState& graph, int from_ix);
    static std::vector<float> dijkstraVarianceAll(const GraphState& graph,
                                                  int from_ix);

    static float graphDist(const GraphState& graph, int from_ix, int to_ix);
    static float graphVarianceDist(const GraphState& graph, int from_ix,
                                   int to_ix);
};

}  // namespace alc_planner
