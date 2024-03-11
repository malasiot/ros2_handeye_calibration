#include "util.hpp"

using namespace Eigen ;

Quaterniond quatFromRPY(double roll, double pitch, double yaw) {
    Quaterniond q ;
    q = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q ;
}

void rpyFromQuat(const Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    Matrix3d r = q.toRotationMatrix() ;
    Vector3d euler = r.eulerAngles(2, 1, 0) ;
    yaw = euler.x() ;
    pitch = euler.y() ;
    roll = euler.z() ;
}

void poseMsgToEigenVectorQuaternion( Vector3d &pos,  Quaterniond &orient, const geometry_msgs::msg::Pose &pose)
{
    pos.x() = pose.position.x ;
    pos.y() = pose.position.y ;
    pos.z() = pose.position.z ;

    orient.x() = pose.orientation.x ;
    orient.y() = pose.orientation.y ;
    orient.z() =  pose.orientation.z ;
    orient.w() = pose.orientation.w ;
}

Eigen::Affine3d poseMsgToEigenAffine( const  geometry_msgs::msg::Pose &pose) {
    Vector3d pos ;
    Quaterniond orient ;
    poseMsgToEigenVectorQuaternion(pos, orient, pose) ;
    Eigen::Affine3d trans ;
    trans.translation() = pos ;
    trans.linear() = orient.toRotationMatrix() ;
    return trans ;
}

void poseEigenToMsg(const Vector3d &pos, const Quaterniond &orient, geometry_msgs::msg::Pose &pose) {
    pose.position.x = pos.x() ;
    pose.position.y = pos.y() ;
    pose.position.z = pos.z() ;

    pose.orientation.x = orient.x() ;
    pose.orientation.y = orient.y() ;
    pose.orientation.z = orient.z() ;
    pose.orientation.w = orient.w() ;
}

void poseEigenAffineToMsg(const Affine3d &pose, geometry_msgs::msg::Pose &msg) {
    Vector3d pos;
    Quaterniond orient ;

    pos = pose.translation() ;
    orient = pose.rotation() ;

    poseEigenToMsg(pos, orient, msg) ;
}


Quaterniond lookAt(const Eigen::Vector3d &dir, double roll) {
     Vector3d nz = dir, na, nb ;
     nz.normalize() ;

     double q = sqrt(nz.x() * nz.x() + nz.y() * nz.y()) ;

     if ( q < 1.0e-4 )
     {
         na = Vector3d(1, 0, 0) ;
         nb = nz.cross(na) ;
     }
     else {
         na = Vector3d(-nz.y()/q, nz.x()/q, 0) ;
         nb = Vector3d(-nz.x() * nz.z()/q, -nz.y() * nz.z()/q, q) ;
     }

     Matrix3d r ;
     r << na, nb, nz ;

     return Quaterniond(r) * AngleAxisd(roll, Eigen::Vector3d::UnitZ()) ;
}
