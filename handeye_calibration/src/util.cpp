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

geometry_msgs::msg::Pose poseEigenToMsg(const Vector3d &pos, const Quaterniond &orient) {
    geometry_msgs::msg::Pose pose;

    pose.position.x = pos.x() ;
    pose.position.y = pos.y() ;
    pose.position.z = pos.z() ;

    pose.orientation.x = orient.x() ;
    pose.orientation.y = orient.y() ;
    pose.orientation.z = orient.z() ;
    pose.orientation.w = orient.w() ;

    return pose ;
}

geometry_msgs::msg::Pose poseEigenAffineToMsg(const Affine3d &pose) {
    geometry_msgs::msg::Pose msg ;

    Vector3d pos;
    Quaterniond orient ;

    pos = pose.translation() ;
    orient = pose.rotation() ;

    return poseEigenToMsg(pos, orient) ;
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

void cameraInfoToCV(const sensor_msgs::msg::CameraInfo::SharedPtr& cam_info, cv::Matx33d& K_, cv::Mat_<double>& D_)  // Unaffected by binning, ROI - they are in ideal camera coordinates
{

  cv::Matx34d P_;  // Describe current image (includes binning, ROI)

  int d_size = cam_info->d.size();
  D_ = (d_size == 0) ? cv::Mat_<double>() : cv::Mat_<double>(1, d_size, cam_info->d.data());
  auto K_full_ = cv::Matx33d(&cam_info->k[0]);
  auto P_full_ = cv::Matx34d(&cam_info->p[0]);

  // Binning = 0 is considered the same as binning = 1 (no binning).
  const uint32_t binning_x = cam_info->binning_x ? cam_info->binning_x : 1;
  const uint32_t binning_y = cam_info->binning_y ? cam_info->binning_y : 1;

  // ROI all zeros is considered the same as full resolution.
  sensor_msgs::msg::RegionOfInterest roi = cam_info->roi;
  if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
    roi.width  = cam_info->width;
    roi.height = cam_info->height;
  }

  // If necessary, create new K_ and P_ adjusted for binning and ROI
  /// @todo Calculate and use rectified ROI
  const bool adjust_binning = (binning_x > 1) || (binning_y > 1);
  const bool adjust_roi = (roi.x_offset != 0) || (roi.y_offset != 0);

  if (!adjust_binning && !adjust_roi) {
    K_ = K_full_;
    P_ = P_full_;
  } else {
    K_ = K_full_;
    P_ = P_full_;

    // ROI is in full image coordinates, so change it first
    if (adjust_roi) {
      // Move principal point by the offset
      /// @todo Adjust P by rectified ROI instead
      K_(0,2) -= roi.x_offset;
      K_(1,2) -= roi.y_offset;
      P_(0,2) -= roi.x_offset;
      P_(1,2) -= roi.y_offset;
    }

    if (binning_x > 1) {
      const double scale_x = 1.0 / binning_x;
      K_(0,0) *= scale_x;
      K_(0,2) *= scale_x;
      P_(0,0) *= scale_x;
      P_(0,2) *= scale_x;
      P_(0,3) *= scale_x;
    }
    if (binning_y > 1) {
      const double scale_y = 1.0 / binning_y;
      K_(1,1) *= scale_y;
      K_(1,2) *= scale_y;
      P_(1,1) *= scale_y;
      P_(1,2) *= scale_y;
      P_(1,3) *= scale_y;
    }
  }
}
