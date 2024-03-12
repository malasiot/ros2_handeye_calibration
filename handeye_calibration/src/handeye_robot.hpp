#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "robot_commander.hpp"
#include "handeye_calibration_msgs/srv/move_robot.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"



class HandEyeRobotCommander:  public rclcpp::Node {
public:
    HandEyeRobotCommander();

    void setup();

    void moveRobot() ;

    void showImage();

    bool estimatePose(Eigen::Affine3d &pose) ;
private:
    void moveRobotCallback(const std::shared_ptr<handeye_calibration_msgs::srv::MoveRobot::Request> request,
                           const std::shared_ptr<handeye_calibration_msgs::srv::MoveRobot::Response> response) ;
 void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
 void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state);
    void updateTransforms(const std::map<std::string, double> &joint_positions, const builtin_interfaces::msg::Time & time);
    void fetchCameraFrame() ;
    void setupModel(const std::string &model_path) ;

private:
    std::shared_ptr<RobotCommander> commander_ ;
    std::shared_ptr<rclcpp::Service<handeye_calibration_msgs::srv::MoveRobot>> move_robot_service_ ;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_ ;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ ;
    std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;
    rclcpp::Time last_callback_time_;
    std::string target_frame_ ;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<Eigen::Affine3d> poses_ ;
    uint current_pose_ = 0;

    std::shared_ptr<image_transport::ImageTransport> image_transport_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> camera_sub_;
    cv::Mat image_ ;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_ = nullptr ;
    float marker_length_ = 0.1 ;

};
