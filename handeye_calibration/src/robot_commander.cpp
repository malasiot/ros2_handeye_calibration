#include "robot_commander.hpp"

RobotCommander::RobotCommander(const std::shared_ptr<rclcpp::Node> &node, const std::string &move_group):
    moveit::planning_interface::MoveGroupInterface(node, move_group) {

}
