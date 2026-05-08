#include <rclcpp/rclcpp.hpp>

#include "alc_planner/planner_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<alc_planner::ALCPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
