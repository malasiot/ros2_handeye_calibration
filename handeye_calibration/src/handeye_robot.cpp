#include "handeye_robot.hpp"
#include "robot_commander.hpp"
#include "util.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <random>
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
HandEyeRobotCommander::HandEyeRobotCommander(): rclcpp::Node("handeye_robot", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto server = std::make_shared<HandEyeRobotCommander>();

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

           current_pose ++ ;


        } else if ( c == 'q' ) break ;
    }

    rclcpp::shutdown();
    spinner.join() ;
}

