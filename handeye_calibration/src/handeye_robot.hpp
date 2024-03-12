#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robot_commander.hpp"
#include "handeye_calibration_msgs/action/move_robot.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp_components/register_node_macro.hpp"



class HandEyeRobotActionServer:  public rclcpp::Node {
public:
    HandEyeRobotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    void setup();


    void showImage();

    bool estimatePose(Eigen::Affine3d &pose) ;
private:

 void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
 void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);
    void updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time & time);
    void fetchCameraFrame() ;
    void setupModel(const std::string &model_path) ;

private:
    using MoveRobot = handeye_calibration_msgs::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

    void moveRobot(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) ;


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::vector<Eigen::Affine3d> poses_ ;
    uint current_pose_ = 0;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> camera_sub_;
    cv::Mat image_ ;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_ = nullptr ;
    float marker_length_ = 0.1 ;

};
RCLCPP_COMPONENTS_REGISTER_NODE(HandEyeRobotActionServer)
