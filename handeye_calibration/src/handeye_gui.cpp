#include "handeye_gui.hpp"
#include <QVBoxLayout>
#include <QApplication>
#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <cv_bridge/cv_bridge.h>
#include <handeye_calibration_msgs/action/move_robot.hpp>
#include <opencv2/opencv.hpp>

#include <QDebug>

using namespace std ;

class HandeyeMoveRobotActionClient: public rclcpp::Node {
public:
    using MoveRobot = handeye_calibration_msgs::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

    HandeyeMoveRobotActionClient(rclcpp::NodeOptions nh = rclcpp::NodeOptions()):
        rclcpp::Node("action_client_node", nh) {
        action_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot") ;
    }

    void setDashBoard(HandeyeCalibrationDashboard *dashboard) {
        dashboard_ = dashboard ;
    }

    void sendMoveToNextGoalToActionServer() {
        using namespace std::placeholders;

        if (!action_client_->wait_for_action_server()) {
          RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();
        }

        auto goal_msg = MoveRobot::Goal();
        goal_msg.request = MoveRobot::Goal::REQUEST_TYPE_MOVE_TO_NEXT_POSITION ;

        RCLCPP_INFO(get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&HandeyeMoveRobotActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
          std::bind(&HandeyeMoveRobotActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
          std::bind(&HandeyeMoveRobotActionClient::result_callback, this, _1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
      }
private:
    void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
      }

      void feedback_callback(
        GoalHandleMoveRobot::SharedPtr,
        const std::shared_ptr<const MoveRobot::Feedback> feedback)
      {
        std::stringstream ss;

        if ( feedback->stage == MoveRobot::Feedback::STAGE_MOVE_TO_INITIALIZING ) {
            ss << "Initializing robot" ;
        } else if ( feedback->stage == MoveRobot::Feedback::STAGE_MOVE_TO_PLANNING_MOTION ) {
            ss << "Planning motion" ;
        } else if ( feedback->stage == MoveRobot::Feedback::STAGE_MOVE_TO_EXECUTING_MOTION ) {
            ss << "Executing motion" ;
        } else if ( feedback->stage == MoveRobot::Feedback::STAGE_MOVE_TO_DETECTING_MARKERS ) {
            ss << "Detecting markers" ;
        }

        string msg = ss.str() ;
        QMetaObject::invokeMethod(dashboard_, [=]() {
            dashboard_->setMessage(msg);
        }, Qt::ConnectionType::QueuedConnection);


        RCLCPP_INFO(get_logger(), msg.c_str());
      }

      void result_callback(const GoalHandleMoveRobot::WrappedResult & result)
      {

          if ( result.code == rclcpp_action::ResultCode::SUCCEEDED ) {
              cv_bridge::CvImagePtr cv_ptr;

              cv_ptr = cv_bridge::toCvCopy(result.result->target_image, result.result->target_image.encoding);
              cv::imwrite("/tmp/im.png", cv_ptr->image) ;

              cv::Mat im = cv_ptr->image ;

              QMetaObject::invokeMethod(dashboard_, [this, im]() {
                  dashboard_->setSuccess(im);
              }, Qt::ConnectionType::QueuedConnection);



          } else if ( result.code == rclcpp_action::ResultCode::ABORTED ) {
              QMetaObject::invokeMethod(dashboard_, [this]() {
                  dashboard_->setFailed();
              }, Qt::ConnectionType::QueuedConnection);
          }

      }

    rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
    HandeyeCalibrationDashboard *dashboard_ ;
 };

RCLCPP_COMPONENTS_REGISTER_NODE(HandeyeMoveRobotActionClient)

HandeyeCalibrationDashboard::HandeyeCalibrationDashboard(HandeyeMoveRobotActionClientPtr nh, QWidget *parent): QWidget(parent), action_client_node_(nh)
{
    run_button_ = new QPushButton(this);
    run_button_->setText("Run") ;

    status_ = new QLabel(this) ;

    image_widget_ = new QImageWidget(this) ;

    QVBoxLayout *layout = new QVBoxLayout() ;
    layout->addWidget(image_widget_) ;
    layout->addWidget(run_button_) ;
    layout->addWidget(status_) ;

    setLayout(layout) ;
    connect(run_button_, &QPushButton::clicked, this, &HandeyeCalibrationDashboard::onRun) ;


}

void HandeyeCalibrationDashboard::setMessage(const std::string &message)
{
    status_->setText(QString::fromStdString(message)) ;
}

void HandeyeCalibrationDashboard::setSuccess(const cv::Mat &im)
{
    status_->setText("Success") ;
    run_button_->setEnabled(true) ;
    image_widget_->setImage(im) ;
}

void HandeyeCalibrationDashboard::setFailed()
{
    status_->setText("Failed") ;
    run_button_->setEnabled(true) ;
}


void HandeyeCalibrationDashboard::onRun()
{
    run_button_->setEnabled(false) ;
    action_client_node_->sendMoveToNextGoalToActionServer();
}



using std::placeholders::_1;



int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    auto server = std::make_shared<HandeyeMoveRobotActionClient>();


    QMainWindow window ;
    HandeyeCalibrationDashboard *dashboard = new HandeyeCalibrationDashboard(server) ;
    window.setCentralWidget(dashboard) ;
    window.resize(512, 512) ;
    window.show() ;

    server->setDashBoard(dashboard) ;
    std::thread t = std::thread([server]{
        rclcpp::spin(server) ;
    });


     app.exec();


    printf("Exited QT thread\n");
    rclcpp::shutdown();


    return 0;
}
