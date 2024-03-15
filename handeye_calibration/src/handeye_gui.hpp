#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QListView>
#include <QStringListModel>

#include <opencv2/opencv.hpp>

#include "image_widget.hpp"

class HandeyeMoveRobotActionClient ;
using HandeyeMoveRobotActionClientPtr = std::shared_ptr<HandeyeMoveRobotActionClient> ;

class HandeyeCalibrationDashboard: public QWidget
{
    Q_OBJECT

public:
    HandeyeCalibrationDashboard(HandeyeMoveRobotActionClientPtr nh, QWidget *parent = 0);

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
    QPushButton *run_button_, *reset_button_, *cal_button_ ;
    QLabel *status_ ;
    QImageWidget *image_widget_ ;
    QStringListModel *model_ ;
    QListView *list_view_ ;
    QVector<int> frame_ids_ ;

 };

