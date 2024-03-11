#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

Eigen::Quaterniond quatFromRPY(double roll, double pitch, double yaw) ;
void rpyFromQuat(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) ;

// a rotation that will align the Z axis with the given direction

Eigen::Quaterniond lookAt(const Eigen::Vector3d &dir, double roll = 0.0) ;


void poseMsgToEigenVectorQuaternion( Eigen::Vector3d &pos,  Eigen::Quaterniond &orient, const geometry_msgs::msg::Pose &pose) ;

Eigen::Affine3d poseMsgToEigenAffine( const  geometry_msgs::msg::Pose &pose) ;

void poseEigenToMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &orient, geometry_msgs::msg::Pose &pose) ;

void poseEigenAffineToMsg(const Eigen::Affine3d &pose, geometry_msgs::msg::Pose &msg) ;
