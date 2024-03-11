#pragma once
#include <moveit/move_group_interface/move_group_interface.h>

class RobotCommander: public moveit::planning_interface::MoveGroupInterface {
public:

    RobotCommander(const std::shared_ptr<rclcpp::Node> &node, const std::string &move_group) ;

private:

    std::string move_group_ ;

};
