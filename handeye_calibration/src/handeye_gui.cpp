#include "handeye_gui.hpp"
#include <QVBoxLayout>
#include <QApplication>
#include <QMainWindow>
#include <QSplitter>
#include <QShortcut>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <cv_bridge/cv_bridge.h>
#include <handeye_calibration_msgs/action/move_robot.hpp>
#include <handeye_calibration_msgs/srv/calibrate.hpp>
#include <opencv2/opencv.hpp>

#include <QDebug>

using namespace std ;

class HandeyeMoveRobotActionClient: public rclcpp::Node {
public:
    using MoveRobot = handeye_calibration_msgs::action::MoveRobot;
    using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;
    using Calibrate = handeye_calibration_msgs::srv::Calibrate ;

    HandeyeMoveRobotActionClient(rclcpp::NodeOptions nh = rclcpp::NodeOptions()):
        rclcpp::Node("action_client_node", nh) {
        action_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot") ;
        calibration_client_ = create_client<Calibrate>("handeye_calibration");
    }

    void setDashBoard(HandeyeCalibrationDashboard *dashboard) {
        dashboard_ = dashboard ;
    }

    void sendCalibrateRequest(const std::vector<int> &frames) {
        auto request = std::make_shared<Calibrate::Request>();
        request->frames = frames ;

        while (! calibration_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
              return ;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = calibration_client_->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            QMetaObject::invokeMethod(dashboard_, [=]() {
                dashboard_->setCalibrationSuccess();
            }, Qt::ConnectionType::QueuedConnection);

          } else {
            QMetaObject::invokeMethod(dashboard_, [=]() {
                dashboard_->setCalibrationFailed();
            }, Qt::ConnectionType::QueuedConnection);


          }
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
          std::bind(&HandeyeMoveRobotActionClient::move_to_result_callback, this, _1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
      }

    void sendResetGoalToActionServer() {
        using namespace std::placeholders;

        if (!action_client_->wait_for_action_server()) {
          RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();
        }

        auto goal_msg = MoveRobot::Goal();
        goal_msg.request = MoveRobot::Goal::REQUEST_TYPE_RESET ;

        RCLCPP_INFO(get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&HandeyeMoveRobotActionClient::goal_response_callback, this, _1);
        send_goal_options.result_callback =
          std::bind(&HandeyeMoveRobotActionClient::reset_result_callback, this, _1);
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

      void move_to_result_callback(const GoalHandleMoveRobot::WrappedResult & result)
      {
          if ( result.code == rclcpp_action::ResultCode::SUCCEEDED ) {
              cv_bridge::CvImagePtr cv_ptr;

              cv_ptr = cv_bridge::toCvCopy(result.result->target_image, result.result->target_image.encoding);

              cv::Mat im = cv_ptr->image ;
              int frame_id = result.result->frame_id ;

              stringstream ss ;
              ss << "/tmp/handeye_frame_" << frame_id << ".png" ;
              cv::imwrite(ss.str(), im) ;

              QMetaObject::invokeMethod(dashboard_, [this, im, frame_id]() {
                  dashboard_->setMoveSuccess(im, frame_id);
              }, Qt::ConnectionType::QueuedConnection);
          } else if ( result.code == rclcpp_action::ResultCode::ABORTED ) {
              QMetaObject::invokeMethod(dashboard_, [this]() {
                  dashboard_->setMoveFailed();
              }, Qt::ConnectionType::QueuedConnection);
          }
      }

      void reset_result_callback(const GoalHandleMoveRobot::WrappedResult & result) {
          if ( result.code == rclcpp_action::ResultCode::SUCCEEDED ) {
              QMetaObject::invokeMethod(dashboard_, [this]() {
                  dashboard_->setResetSuccess();
              }, Qt::ConnectionType::QueuedConnection);
          } else if ( result.code == rclcpp_action::ResultCode::ABORTED ) {
              QMetaObject::invokeMethod(dashboard_, [this]() {
                  dashboard_->setResetFailed();
              }, Qt::ConnectionType::QueuedConnection);
          }

      }

    rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
    rclcpp::Client<Calibrate>::SharedPtr calibration_client_ ;

    HandeyeCalibrationDashboard *dashboard_ ;
 };

RCLCPP_COMPONENTS_REGISTER_NODE(HandeyeMoveRobotActionClient)

HandeyeCalibrationDashboard::HandeyeCalibrationDashboard(HandeyeMoveRobotActionClientPtr nh, QWidget *parent): QWidget(parent), action_client_node_(nh)
{
    run_button_ = new QPushButton(this);
    run_button_->setText("Run") ;

    reset_button_ = new QPushButton(this);
    reset_button_->setText("Reset") ;

    cal_button_ = new QPushButton(this);
    cal_button_->setText("Calibrate") ;

    status_ = new QLabel(this) ;

    QSplitter *hbox1 = new QSplitter(Qt::Horizontal, this) ;


    list_view_ = new QListView(this) ;
    image_widget_ = new QImageWidget(this) ;

    model_ = new QStringListModel(this);
    list_view_->setModel(model_);

    hbox1->addWidget(list_view_) ;
    hbox1->addWidget(image_widget_) ;
     hbox1->setSizes({200, 400});

    QHBoxLayout *hbox = new QHBoxLayout() ;
    hbox->addWidget(reset_button_) ;
    hbox->addWidget(run_button_) ;
    hbox->addWidget(cal_button_) ;

    QVBoxLayout *layout = new QVBoxLayout() ;
    layout->addWidget(hbox1, 1) ;

    layout->addLayout(hbox) ;
    layout->addWidget(status_) ;

    setLayout(layout) ;
    connect(run_button_, &QPushButton::clicked, this, &HandeyeCalibrationDashboard::onRun) ;
    connect(reset_button_, &QPushButton::clicked, this, &HandeyeCalibrationDashboard::onReset) ;


    QShortcut* shortcut = new QShortcut(QKeySequence(Qt::Key_Delete), list_view_);
    connect(shortcut, SIGNAL(activated()), this, SLOT(deleteItem()));

    connect(list_view_->selectionModel(), &QItemSelectionModel::currentChanged, this, &HandeyeCalibrationDashboard::displayFrame) ;
}

void HandeyeCalibrationDashboard::setMessage(const std::string &message)
{
    status_->setText(QString::fromStdString(message)) ;
}

void HandeyeCalibrationDashboard::setMoveSuccess(const cv::Mat &im, int frame_id) {
    status_->setText("Success") ;
    image_widget_->setImage(im) ;
    if( model_->insertRow(model_->rowCount())) {
        QModelIndex index = model_->index(model_->rowCount() - 1, 0);
        model_->setData(index, QString("Frame %1").arg(frame_id));
        frame_ids_.append(frame_id) ;
        list_view_->setCurrentIndex(index) ;
    }
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::setMoveFailed() {
    status_->setText("Failed") ;
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::setResetSuccess() {
    status_->setText("Success") ;
    model_->setStringList({}) ;
    frame_ids_.clear() ;
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::setResetFailed() {
    status_->setText("Failed") ;
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::setCalibrationSuccess()
{
    status_->setText("Success") ;
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::setCalibrationFailed()
{
    status_->setText("Failed") ;
    enableButtons(true) ;
}

void HandeyeCalibrationDashboard::onRun()
{
    enableButtons(false) ;
    action_client_node_->sendMoveToNextGoalToActionServer();
}

void HandeyeCalibrationDashboard::onReset()
{
    enableButtons(false) ;
    action_client_node_->sendResetGoalToActionServer();
}

void HandeyeCalibrationDashboard::onCalibrate() {
    enableButtons(false) ;

    vector<int> frames ;
    for( const auto &frame: frame_ids_ ) {
        frames.push_back(frame) ;
    }
    action_client_node_->sendCalibrateRequest(frames) ;
}

void HandeyeCalibrationDashboard::deleteItem() {
    int row = list_view_->currentIndex().row() ;
    model_->removeRow(row);

    frame_ids_.remove(row) ;
}

void HandeyeCalibrationDashboard::displayFrame(const QModelIndex &index)
{
    stringstream ss ;
    ss << "/tmp/handeye_frame_" << frame_ids_[index.row()] << ".png" ;
    cv::Mat im = cv::imread(ss.str()) ;
    image_widget_->setImage(im) ;
}

void HandeyeCalibrationDashboard::enableButtons(bool enable) {
    run_button_->setEnabled(enable) ;
    reset_button_->setEnabled(enable) ;
    cal_button_->setEnabled(enable) ;
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
