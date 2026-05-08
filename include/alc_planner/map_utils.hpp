#pragma once

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <vector>

#include "alc_planner/types.hpp"

namespace alc_planner
{

struct GridCell
{
    int row = 0;
    int col = 0;
};

inline bool inBounds(const GridCell& cell, const int width, const int height) {
    return cell.row >= 0 && cell.row < height && cell.col >= 0 &&
           cell.col < width;
}

inline int toIndex(const GridCell& cell, const int width) {
    return cell.row * width + cell.col;
}

inline GridCell toCell(const Eigen::Vector3f& pos,
                       const nav_msgs::msg::OccupancyGrid& map) {
    const float resolution = map.info.resolution;
    const float origin_x = static_cast<float>(map.info.origin.position.x);
    const float origin_y = static_cast<float>(map.info.origin.position.y);
    return {
        static_cast<int>(std::floor((pos.y() - origin_y) / resolution)),
        static_cast<int>(std::floor((pos.x() - origin_x) / resolution)),
    };
}

float estimateCoverageRatio(const nav_msgs::msg::OccupancyGrid* map);

void buildSaliencyOverlay(const GraphState& graph,
                          const SaliencyState& saliency_state,
                          const nav_msgs::msg::OccupancyGrid& map,
                          std::vector<float>& overlay);

}  // namespace alc_planner
