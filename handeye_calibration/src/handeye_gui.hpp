#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>

#include <opencv2/opencv.hpp>

class HandeyeMoveRobotActionClient ;
using HandeyeMoveRobotActionClientPtr = std::shared_ptr<HandeyeMoveRobotActionClient> ;

class HandeyeCalibrationDashboard: public QWidget
{
    Q_OBJECT

public:
    HandeyeCalibrationDashboard(HandeyeMoveRobotActionClientPtr nh, QWidget *parent = 0);

    void setMessage(const std::string &message) ;
    void setSuccess(const cv::Mat &im) ;
    void setFailed() ;

private Q_SLOTS:
    void onRun() ;
private:
    HandeyeMoveRobotActionClientPtr action_client_node_ ;
    QPushButton *run_button_ ;
    QLabel *status_ ;

 };

