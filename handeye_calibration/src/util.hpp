#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) ;
void rpyFromQuat(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) ;

// a rotation that will align the Z axis with the given direction

Eigen::Quaterniond lookAt(const Eigen::Vector3d &dir, double roll = 0.0) ;


void poseMsgToEigenVectorQuaternion( Eigen::Vector3d &pos,  Eigen::Quaterniond &orient, const geometry_msgs::msg::Pose &pose) ;

Eigen::Affine3d poseMsgToEigenAffine( const  geometry_msgs::msg::Pose &pose) ;

geometry_msgs::msg::Pose poseEigenToMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient) ;

geometry_msgs::msg::Pose  poseEigenAffineToMsg(const Eigen::Affine3d &pose) ;

void cameraInfoToCV(const sensor_msgs::msg::CameraInfo::SharedPtr& msg,
    cv::Matx33d& K_,  // Describe current image (includes binning, ROI)
    cv::Mat_<double>& D_)  ;// Unaffected by binning, ROI - they are in ideal camera coordinates
