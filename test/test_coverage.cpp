#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "alc_planner/map_utils.hpp"

namespace alc_planner
{
namespace
{

nav_msgs::msg::OccupancyGrid buildMap(const int width, const int height) {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = width;
    map.info.height = height;
    map.info.resolution = 1.0f;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(width * height), -1);
    return map;
}

}  // namespace

TEST(CoverageRatio, NullMapFallsBackToNeutral) {
    EXPECT_FLOAT_EQ(estimateCoverageRatio(nullptr), 0.5f);
}

TEST(CoverageRatio, EmptyMapFallsBackToNeutral) {
    nav_msgs::msg::OccupancyGrid map;
    EXPECT_FLOAT_EQ(estimateCoverageRatio(&map), 0.5f);
}

TEST(CoverageRatio, FullyFreeMapReturnsOne) {
    auto map = buildMap(4, 4);
    std::fill(map.data.begin(), map.data.end(), 0);
    EXPECT_FLOAT_EQ(estimateCoverageRatio(&map), 1.0f);
}

TEST(CoverageRatio, FullyUnknownMapReturnsZero) {
    auto map = buildMap(4, 4);
    EXPECT_FLOAT_EQ(estimateCoverageRatio(&map), 0.0f);
}

TEST(CoverageRatio, FullyOccupiedMapFallsBackToNeutral) {
    auto map = buildMap(4, 4);
    std::fill(map.data.begin(), map.data.end(), 100);
    EXPECT_FLOAT_EQ(estimateCoverageRatio(&map), 0.5f);
}

TEST(CoverageRatio, MixedDiscoverableCellsReturnExpectedRatio) {
    auto map = buildMap(10, 10);
    for (int i = 0; i < 30; ++i) {
        map.data[static_cast<std::size_t>(i)] = 0;
    }
    for (int i = 30; i < 100; ++i) {
        map.data[static_cast<std::size_t>(i)] = -1;
    }
    EXPECT_NEAR(estimateCoverageRatio(&map), 0.3f, 1e-4f);
}

}  // namespace alc_planner
