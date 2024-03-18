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
#include "handeye_calibration_msgs/srv/calibrate.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp_components/register_node_macro.hpp"



class HandEyeRobotActionServer:  public rclcpp::Node {
public:
    HandEyeRobotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    void setup();


    void showImage();

    bool estimatePose(Eigen::Affine3d &pose, cv::Mat &out) ;

private:

 void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
 void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);



private:
    using MoveRobot = handeye_calibration_msgs::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;
    using Calibrate = handeye_calibration_msgs::srv::Calibrate ;

    rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
    rclcpp::Service<Calibrate>::SharedPtr calibration_service_ ;

    void moveRobot(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) ;
    void resetRobot(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) ;
    void calibrate(const std::shared_ptr<Calibrate::Request> request, std::shared_ptr<Calibrate::Response> response) ;

    void exportSamples() ;

    std::vector<Eigen::Affine3d> poses_ ;
    uint current_pose_ = 0;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> camera_sub_;
    cv::Mat image_ ;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_ = nullptr ;

    std::string camera_info_topic_, image_topic_, ee_link_, move_group_, robot_start_state_name_, aruco_dict_ ;
    uint32_t markers_x_, markers_y_ ;
    double marker_length_, marker_separation_ ;

    struct CalibrationDataSample {
        Eigen::Affine3d cam2target_ ;
        Eigen::Affine3d base2gripper_ ;
        int frame_id_ ;
    };

    std::vector<CalibrationDataSample> samples_ ;

};
RCLCPP_COMPONENTS_REGISTER_NODE(HandEyeRobotActionServer)
