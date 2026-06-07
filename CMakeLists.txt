cmake_minimum_required(VERSION 3.8)
project(alc_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

add_library(alc_planner_core
  src/core/map_utils.cpp
  src/core/slam_graph_planner.cpp
  src/core/saliency_evaluator.cpp
  src/core/uncertainty_metrics.cpp
  src/core/candidate_builder.cpp
  src/core/reward_evaluator.cpp
  src/core/bnb_selector.cpp
  src/core/path_planner.cpp
  src/core/trigger_policy.cpp
)

target_include_directories(alc_planner_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(alc_planner_core PUBLIC cxx_std_17)
target_link_libraries(alc_planner_core Eigen3::Eigen)
ament_target_dependencies(
  alc_planner_core
  eigen3_cmake_module
  nav_msgs
)

add_library(alc_planner_node_lib src/node/planner_node.cpp)
target_include_directories(alc_planner_node_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(alc_planner_node_lib PUBLIC cxx_std_17)
target_link_libraries(alc_planner_node_lib alc_planner_core)
ament_target_dependencies(
  alc_planner_node_lib
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rtabmap_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_executable(alc_planner_node src/node/alc_planner_node_main.cpp)
target_compile_features(alc_planner_node PUBLIC cxx_std_17)
target_link_libraries(alc_planner_node alc_planner_node_lib)
ament_target_dependencies(alc_planner_node rclcpp)

add_executable(db_publisher_node test/db_publisher_node.cpp)
target_compile_features(db_publisher_node PUBLIC cxx_std_17)
target_link_libraries(db_publisher_node alc_planner_core ${SQLITE3_LIBRARIES})
target_include_directories(db_publisher_node PRIVATE ${SQLITE3_INCLUDE_DIRS})
ament_target_dependencies(db_publisher_node nav_msgs rclcpp rtabmap_msgs)

install(
  TARGETS alc_planner_core
  EXPORT export_alc_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS alc_planner_node_lib alc_planner_node db_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch docs DESTINATION share/${PROJECT_NAME})
install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_alc_planner_core HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_uncertainty_metrics test/test_uncertainty_metrics.cpp)
  target_link_libraries(test_uncertainty_metrics alc_planner_core)

  ament_add_gtest(test_saliency_evaluator test/test_saliency_evaluator.cpp)
  target_link_libraries(test_saliency_evaluator alc_planner_core)

  ament_add_gtest(test_candidate_builder test/test_candidate_builder.cpp)
  target_link_libraries(test_candidate_builder alc_planner_core)

  ament_add_gtest(test_reward_evaluator test/test_reward_evaluator.cpp)
  target_link_libraries(test_reward_evaluator alc_planner_core)

  ament_add_gtest(test_path_planner test/test_path_planner.cpp)
  target_link_libraries(test_path_planner alc_planner_core)

  ament_add_gtest(test_bnb_selector test/test_bnb_selector.cpp)
  target_link_libraries(test_bnb_selector alc_planner_core)

  ament_add_gtest(test_trigger_policy test/test_trigger_policy.cpp)
  target_link_libraries(test_trigger_policy alc_planner_core)

  ament_add_gtest(test_planner_node_params test/test_planner_node_params.cpp)
  target_link_libraries(test_planner_node_params alc_planner_node_lib)
  ament_target_dependencies(test_planner_node_params rclcpp)

  ament_add_gtest(test_coverage test/test_coverage.cpp)
  target_link_libraries(test_coverage alc_planner_core)
endif()

ament_package()
