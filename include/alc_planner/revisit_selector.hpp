#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <optional>
#include <vector>

#include "alc_planner/path_planner.hpp"
#include "alc_planner/types.hpp"

namespace alc_planner
{

struct RevisitDecision
{
    // Largest position error, over all old keyframes, of the robot relative to
    // that keyframe. Logged whether or not anything is worth a revisit.
    float max_position_std = 0.0f;
    std::optional<ALCCandidate> best;
};

// Chooses the old keyframe whose recognition would remove the most pose
// uncertainty per metre driven. Uncertainty comes from the whole pose graph
// (odometry drift per metre and per radian, closures), so it needs no length
// scale: a longer, more winding graph is simply more uncertain.
class RevisitSelector
{
public:
    explicit RevisitSelector(Params params);

    // `excluded` holds targets that were visited without a closure.
    RevisitDecision select(const GraphState& graph,
                           const SaliencyState& saliency,
                           const nav_msgs::msg::OccupancyGrid& map,
                           const std::vector<Pose6f>& excluded) const;

private:
    Params params_;
    PathPlanner path_planner_;
};

}  // namespace alc_planner
