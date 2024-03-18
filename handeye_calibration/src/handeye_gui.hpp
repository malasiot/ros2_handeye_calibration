#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QListView>
#include <QStringListModel>
#include <QComboBox>

#include <opencv2/opencv.hpp>

#include "image_widget.hpp"

class HandeyeMoveRobotActionClient ;
using HandeyeMoveRobotActionClientPtr = std::shared_ptr<HandeyeMoveRobotActionClient> ;

class HandeyeCalibrationClient ;
using HandeyeCalibrationClientPtr = std::shared_ptr<HandeyeCalibrationClient> ;

class HandeyeCalibrationDashboard: public QWidget
{
    Q_OBJECT

public:
    HandeyeCalibrationDashboard(HandeyeMoveRobotActionClientPtr nh, HandeyeCalibrationClientPtr ch, QWidget *parent = 0);

    void setMessage(const std::string &message) ;
    void setMoveSuccess(const cv::Mat &im, int frame_id) ;
    void setMoveFailed() ;

    void setResetSuccess() ;
    void setResetFailed() ;

    void setCalibrationSuccess() ;
    void setCalibrationFailed() ;

private Q_SLOTS:
    void onRun() ;
    void onReset() ;
    void onCalibrate() ;
    void deleteItem() ;
    void displayFrame(const QModelIndex &index) ;

private:
    void enableButtons(bool enable) ;

    HandeyeMoveRobotActionClientPtr action_client_node_ ;
    HandeyeCalibrationClientPtr calibration_client_node_ ;

    QPushButton *run_button_, *reset_button_, *cal_button_ ;
    QLabel *status_ ;
    QComboBox *algorithm_ ;
    QImageWidget *image_widget_ ;
    QStringListModel *model_ ;
    QListView *list_view_ ;
    QVector<int> frame_ids_ ;

 };

