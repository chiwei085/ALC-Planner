#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <optional>
#include <vector>

#include "alc_planner/path_planner.hpp"
#include "alc_planner/reward_evaluator.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{

class BNBSelector
{
public:
    explicit BNBSelector(Params params);

    std::optional<ALCCandidate> select(
        std::vector<ALCCandidate> candidates, const GraphState& graph,
        const nav_msgs::msg::OccupancyGrid& map) const;

private:
    Params params_;
    RewardEvaluator evaluator_;
    PathPlanner path_planner_;
};

}  // namespace alc_planner
