#include "handeye_gui.hpp"
#include <QVBoxLayout>
#include <QApplication>
#include <QMainWindow>

using namespace std ;

HandeyeCalibrationDashboard::HandeyeCalibrationDashboard(rclcpp::Node::ConstSharedPtr nh, QWidget *parent): QWidget(parent), handle_(nh)
{
    run_button_ = new QPushButton(this);
    run_button_->setText("Run") ;

    QVBoxLayout *layout = new QVBoxLayout() ;
    layout->addWidget(run_button_) ;
    setLayout(layout) ;
    connect(run_button_, &QPushButton::clicked, this, &HandeyeCalibrationDashboard::onRun) ;

}

void HandeyeCalibrationDashboard::onRun()
{

}



using std::placeholders::_1;



int main(int argc, char * argv[])
{
    QApplication app(argc, argv) ;

    rclcpp::init(argc, argv);

    auto server = std::make_shared<rclcpp::Node>("node");


    std::thread t = std::thread([server]{
        rclcpp::spin(server) ;
    });

    QMainWindow window ;
    HandeyeCalibrationDashboard *dashboard = new HandeyeCalibrationDashboard(server) ;
    window.setCentralWidget(dashboard) ;
    window.resize(1024, 1024) ;
    window.show() ;

     app.exec();

     t.join() ;
    printf("Exited QT thread\n");
    rclcpp::shutdown();


    return 0;
}
