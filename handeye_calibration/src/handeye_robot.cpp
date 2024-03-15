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

std::vector<Affine3d> compute_poses_around_current_state(const Affine3d &pose, size_t n_samples, double angle_delta = 0.2, double translation_delta = 0.2) {
    std::uniform_real_distribution<double> ur(-angle_delta, angle_delta);
    std::uniform_real_distribution<double> ut(-translation_delta, translation_delta);
    vector<Affine3d> samples ;

    samples.push_back(pose) ;

    for( uint iter = 0 ; iter < n_samples ; iter ++ ) {
        double r, p, y ;
        rpyFromQuat(Quaterniond(pose.rotation()), r, p, y) ;

     //   r += ur(g_rng) ;  p += ur(g_rng) ; y += ur(g_rng) ;

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
HandEyeRobotActionServer::HandEyeRobotActionServer(const rclcpp::NodeOptions & options):rclcpp::Node("handeye_robot", options) {

}

void HandEyeRobotActionServer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    image_ = cv_ptr->image ;
}

void HandEyeRobotActionServer::setup()
{
    this->action_server_ = rclcpp_action::create_server<MoveRobot>(
         this,
         "move_robot",
                []( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveRobot::Goal> goal) -> rclcpp_action::GoalResponse {
                    (void)uuid;
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
    [](const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
        if ( goal_handle->get_goal()->request == MoveRobot::Goal::REQUEST_TYPE_MOVE_TO_NEXT_POSITION )
            std::thread(&HandEyeRobotActionServer::moveRobot, this, goal_handle).detach() ;
        else if ( goal_handle->get_goal()->request == MoveRobot::Goal::REQUEST_TYPE_RESET )
            std::thread(&HandEyeRobotActionServer::resetRobot, this, goal_handle).detach() ;
    }) ;

    calibration_service_ = create_service<Calibrate>("handeye_calibration", std::bind(&HandEyeRobotActionServer::calibrate, this, std::placeholders::_1, std::placeholders::_2));


    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport_->subscribe("/virtual_camera/color/image_raw", 10, std::bind(&HandEyeRobotActionServer::imageCallback, this, std::placeholders::_1)));

    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/virtual_camera/color/camera_info", 10, std::bind(&HandEyeRobotActionServer::cameraInfoCallback, this, std::placeholders::_1));

}

void HandEyeRobotActionServer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    camera_info_ = msg ;
}

void HandEyeRobotActionServer::moveRobot(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
    auto result = std::make_shared<MoveRobot::Result>();
    auto feedback = std::make_shared<MoveRobot::Feedback>();

    feedback->stage = MoveRobot::Feedback::STAGE_MOVE_TO_INITIALIZING ;
    goal_handle->publish_feedback(feedback);

    std::string move_group = get_parameter_or<std::string>("move_group", "r_iiwa_arm") ;
    std::shared_ptr<RobotCommander> commander(new RobotCommander(shared_from_this(), move_group));

    if ( poses_.empty() ) {
        auto ee_pose = commander->getCurrentPose("r_tool0") ;
        Affine3d ep = poseMsgToEigenAffine(ee_pose.pose);
        poses_ = compute_poses_around_current_state(ep, 100) ;
    }
    feedback->stage = MoveRobot::Feedback::STAGE_MOVE_TO_PLANNING_MOTION ;
    goal_handle->publish_feedback(feedback);

    auto pose = poses_[current_pose_] ;

    geometry_msgs::msg::Pose target_pose = poseEigenAffineToMsg(pose) ;

    commander->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    commander->setEndEffectorLink("r_tool0") ;

    commander->setStartStateToCurrentState();
    auto const ok = static_cast<bool>(commander->plan(plan));

    feedback->stage = MoveRobot::Feedback::STAGE_MOVE_TO_EXECUTING_MOTION ;
    goal_handle->publish_feedback(feedback);

    // Execute the plan
    if(ok) {
        commander->execute(plan);
    } else {
         goal_handle->abort(result);
          current_pose_ ++ ;
         return ;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planning failed!");
    }

    feedback->stage = MoveRobot::Feedback::STAGE_MOVE_TO_DETECTING_MARKERS;
    goal_handle->publish_feedback(feedback);

    Affine3d target_to_camera ;
    cv::Mat viz ;
    if ( estimatePose(target_to_camera, viz) ) {
        cout << target_to_camera.matrix() << endl ;

        result->target_to_camera = poseEigenAffineToMsg(target_to_camera) ;

        result->ee_to_base = commander->getCurrentPose("r_tool0").pose ;

        sensor_msgs::msg::Image::SharedPtr image_msg =
                   cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz)
                       .toImageMsg();
        result->target_image = *image_msg ;

        result->frame_id = samples_.size() ;

        goal_handle->succeed(result);

        CalibrationDataSample sample ;
        sample.frame_id_ = current_pose_ ;
        sample.cam2target_ = target_to_camera ;
        sample.base2gripper_ = poseMsgToEigenAffine(result->ee_to_base) ;
        samples_.emplace_back(sample) ;
    }
    else
        goal_handle->abort(result);
    current_pose_ ++ ;
}

void HandEyeRobotActionServer::resetRobot(const std::shared_ptr<GoalHandleMoveRobot> goal_handle)
{
    std::string move_group = get_parameter_or<std::string>("move_group", "r_iiwa_arm") ;
    std::shared_ptr<RobotCommander> commander(new RobotCommander(shared_from_this(), move_group));

    auto result = std::make_shared<MoveRobot::Result>();
    commander->setNamedTarget("calibration") ;
    if ( commander->move() == moveit::core::MoveItErrorCode::SUCCESS ) {
        poses_.clear() ;
        auto ee_pose = commander->getCurrentPose("r_tool0") ;
        Affine3d ep = poseMsgToEigenAffine(ee_pose.pose);
        poses_ = compute_poses_around_current_state(ep, 100) ;
        current_pose_ = 0 ;
        samples_.clear() ;
        goal_handle->succeed(result) ;
    } else {
        goal_handle->abort(result) ;
    }
}

void HandEyeRobotActionServer::calibrate(const std::shared_ptr<Calibrate::Request> request, std::shared_ptr<Calibrate::Response> response)
{

}

bool HandEyeRobotActionServer::estimatePose(Eigen::Affine3d &pose, cv::Mat &output_image)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::Dictionary> dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));

    cv::aruco::detectMarkers(image_, dictionary, markerCorners, markerIds, cv::aruco::DetectorParameters::create(), rejectedCandidates);

    //cv::imwrite("/tmp/im.png", image_) ;
    if ( markerIds.size() > 0 ) {
        output_image = image_.clone();
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

        if ( camera_info_ ) {
            cv::Matx33d cam ;
            cv::Mat_<double> dist ;
            cameraInfoToCV(camera_info_, cam, dist) ;

            const int markersX = 3 ;
            const int markersY = 3 ;
            const double markerLength = 400 * 0.2 / 1520 ;
            const double markerSeparation = 80 * 0.2/1520 ;

            cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
            cv::aruco::refineDetectedMarkers(image_, board, markerCorners, markerIds, rejectedCandidates, cam, dist);

            cv::Vec3d rvec, tvec ;
            cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cam, dist, rvec, tvec) ;
            cv::drawFrameAxes(output_image, cam, dist, rvec, tvec, 0.1);


            Matrix3d r ;
            Vector3d t ;

            cv::Mat rmat ;
            cv::Rodrigues(rvec, rmat) ;

            cv::Mat_<double> rm(rmat), tmat(tvec) ;

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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto server = std::make_shared<HandEyeRobotActionServer>();
    server->setup() ;
   // server->reset() ;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    spinner.join() ;
    rclcpp::shutdown();

}

