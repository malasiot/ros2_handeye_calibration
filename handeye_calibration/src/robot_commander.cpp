#include "robot_commander.hpp"

RobotCommander::RobotCommander(const std::shared_ptr<rclcpp::Node> &node, const std::string &move_group):
    moveit::planning_interface::MoveGroupInterface(node, move_group) {
    setPlannerId("RRTConnectkConfigDefault"); //default planener
    setPlanningTime(5);

    setGoalTolerance(0.01);
    setMaxAccelerationScalingFactor(1);
    setMaxVelocityScalingFactor(1);
    setNumPlanningAttempts(10);
    setPlanningTime(5);
}
