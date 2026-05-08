#include "alc_planner/map_utils.hpp"

#include <algorithm>

namespace alc_planner
{

float estimateCoverageRatio(const nav_msgs::msg::OccupancyGrid* map) {
    if (map == nullptr || map->data.empty()) {
        return 0.5f;
    }

    int free_count = 0;
    int unknown_count = 0;
    for (const int8_t cell : map->data) {
        if (cell == 0) {
            ++free_count;
        }
        else if (cell < 0) {
            ++unknown_count;
        }
    }

    const int discoverable = free_count + unknown_count;
    if (discoverable <= 0) {
        return 0.5f;
    }

    return static_cast<float>(free_count) / static_cast<float>(discoverable);
}

void buildSaliencyOverlay(const GraphState& graph,
                          const SaliencyState& saliency_state,
                          const nav_msgs::msg::OccupancyGrid& map,
                          std::vector<float>& overlay) {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0 || map.info.resolution <= 0.0f) {
        overlay.clear();
        return;
    }

    overlay.assign(static_cast<std::size_t>(width * height), 0.0f);
    const std::size_t count =
        std::min(graph.keyframes.size(), saliency_state.keyframes.size());
    for (std::size_t i = 0; i < count; ++i) {
        const auto& keyframe = graph.keyframes[i];
        const GridCell cell = toCell(keyframe.pose.position, map);
        if (!inBounds(cell, width, height)) {
            continue;
        }

        float& value = overlay[static_cast<std::size_t>(toIndex(cell, width))];
        value = std::max(value, saliency_state.keyframes[i].saliency_local);
    }
}

}  // namespace alc_planner
