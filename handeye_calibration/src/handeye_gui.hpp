#pragma once

#include <QWidget>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

class HandeyeCalibrationDashboard: public QWidget
{
    Q_OBJECT

public:

    HandeyeCalibrationDashboard(rclcpp::Node::ConstSharedPtr nh, QWidget *parent = 0);

private Q_SLOTS:

    void onRun() ;

private:
    rclcpp::Node::ConstSharedPtr handle_ ;
    QPushButton *run_button_ ;

 };
