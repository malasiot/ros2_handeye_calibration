#include "handeye_robot.hpp"
#include "robot_commander.hpp"
#include "util.hpp"
#include <opencv2/highgui.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <random>

#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std ;
using namespace Eigen ;

std::random_device rd;
std::mt19937 g_rng(rd());

std::vector<Affine3d> compute_poses_around_current_state(const Affine3d &pose, size_t n_samples, double angle_delta = 0.1, double translation_delta = 0.1) {
    std::uniform_real_distribution<double> ur(-angle_delta, angle_delta);
    std::uniform_real_distribution<double> ut(-translation_delta, translation_delta);
    vector<Affine3d> samples ;

    samples.push_back(pose) ;

    for( uint iter = 0 ; iter < n_samples ; iter ++ ) {
        double r, p, y ;
        rpyFromQuat(Quaterniond(pose.rotation()), r, p, y) ;

        r += ur(g_rng) ;  p += ur(g_rng) ; y += ur(g_rng) ;

        auto q = quatFromRPY(r, p, y) ;

        double X = pose.translation().x() ;
        double Y = pose.translation().y() ;
        double Z = pose.translation().z() ;

        X += ut(g_rng) ; Y += ut(g_rng) ; Z += ut(g_rng) ;

        Affine3d sample ;
        sample.linear() = q.matrix() ;
        sample.translation() = Vector3d(X, Y, Z) ;

        samples.emplace_back(sample) ;
    }

    return samples ;

}
HandEyeRobotCommander::HandEyeRobotCommander():rclcpp::Node("handeye_robot", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

}

void HandEyeRobotCommander::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    image_ = cv_ptr->image ;
}

void HandEyeRobotCommander::setup()
{

    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport_->subscribe("/image", 10, std::bind(&HandEyeRobotCommander::imageCallback, this, std::placeholders::_1)));

    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", 10, std::bind(&HandEyeRobotCommander::cameraInfoCallback, this, std::placeholders::_1));

}

void HandEyeRobotCommander::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    camera_info_ = msg ;




}

void HandEyeRobotCommander::moveRobot()
{

}

void HandEyeRobotCommander::showImage() {
    try {



        } catch (cv_bridge::Exception &e) {
          auto logger = rclcpp::get_logger("my_subscriber");
     //     RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.",
     //                  msg->encoding.c_str());
    }
}

bool HandEyeRobotCommander::estimatePose(Eigen::Affine3d &pose)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::Dictionary> dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

    cv::aruco::detectMarkers(image_, dictionary, markerCorners, markerIds, cv::aruco::DetectorParameters::create(), rejectedCandidates);

    //cv::imwrite("/tmp/im.png", image_) ;
    if ( markerIds.size() > 0 ) {
        cv::Mat output_image = image_.clone();
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

        if ( camera_info_ ) {
            cv::Matx33d cam ;
            cv::Mat_<double> dist ;
            cameraInfoToCV(camera_info_, cam, dist) ;

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, cam, dist, rvecs, tvecs);

            for (size_t i = 0; i < rvecs.size(); ++i) {
                auto rvec = rvecs[i];
                auto tvec = tvecs[i];
                cv::drawFrameAxes(output_image, cam, dist, rvec, tvec, 0.1);
            }

            Matrix3d r ;
            Vector3d t ;

            cv::Mat rmat ;
            cv::Rodrigues(rvecs[0], rmat) ;

            cv::Mat_<double> rm(rmat), tmat(tvecs[0]) ;

            r << rm(0, 0), rm(0, 1), rm(0, 2), rm(1, 0), rm(1, 1), rm(1, 2), rm(2, 0), rm(2, 1), rm(2, 2) ;
            t << tmat(0, 0), tmat(0, 1), tmat(0, 2) ;

            pose = Translation3d(t) * r ;
            //cv::imwrite("/tmp/markers.png", output_image) ;

            return true ;

         //   int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cam, dist, rvec, tvec);
        }

    }

    return false ;
}

void HandEyeRobotCommander::moveRobotCallback(const std::shared_ptr<handeye_calibration_msgs::srv::MoveRobot::Request> request, const std::shared_ptr<handeye_calibration_msgs::srv::MoveRobot::Response> response)
{
    std::thread(&HandEyeRobotCommander::moveRobot, this).detach() ;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto server = std::make_shared<HandEyeRobotCommander>();
    server->setup() ;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    std::string move_group = server->get_parameter_or<std::string>("move_group", "r_iiwa_arm") ;
    std::shared_ptr<RobotCommander> commander(new RobotCommander(server, move_group));

    auto ee_pose = commander->getCurrentPose("r_tool0") ;
    Affine3d ep = poseMsgToEigenAffine(ee_pose.pose);
    auto poses = compute_poses_around_current_state(ep, 100) ;

    uint current_pose = 0 ;
    while ( 1 ) {
        cout << "Press <enter> to move to a new position or <q> to quit" << endl ;
        char c = getchar();
        if ( c == '\n') {
            auto pose = poses[current_pose] ;

            geometry_msgs::msg::Pose target_pose ;

            poseEigenAffineToMsg(pose, target_pose) ;

            commander->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            commander->setEndEffectorLink("r_tool0") ;

            commander->setStartStateToCurrentState();
            auto const ok = static_cast<bool>(commander->plan(plan));

            // Execute the plan
            if(ok) {
                commander->execute(plan);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed!");
            }

            Affine3d camera_to_target ;
            if ( server->estimatePose(camera_to_target) ) {
                cout << camera_to_target.matrix() << endl ;
            }
            current_pose ++ ;


        } else if ( c == 'q' ) break ;
    }

    spinner.join() ;
    rclcpp::shutdown();

}

