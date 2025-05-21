#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QTimer>
#include <QPixmap>
#include <QMutex>
#include <QQueue>

#include <opencv2/opencv.hpp>

// #include "OrbSensor.h"

using namespace cv;

class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(QObject *parent = nullptr);

    void openCamera();      // 打开相机
    void closeCamera();     // 关闭相机
    void PreStart();
    void Camerastop();
    QTimer *yoloTimer;       // 采集图像定时器

private:
    VideoCapture *cap;     // 相机
    // OrbSensor * color_camera;
    // OrbSensor * depth_camera;
    // OrbSensor * all_camera;

    QTimer *capTimer;       // 采集图像定时器
    
    bool cameraIsOpened = false;        // 相机打开标志位
    bool detectPlate = true;

    // std::queue<cv::Mat> color_frame_queue;
    QQueue<cv::Mat> color_frame_queue;
    mutable QMutex frame_lock;
    const int MAX_FRAMES = 1;
    
    bool findPlate = false;
    cv::Rect MoveDetect(cv::Mat temp, cv::Mat frame);
    int emptyCount = 0;

signals:
    void cameraShowImage(QPixmap);      // 发送图像信号
    void cameraIsOpen(bool);            // 相机打开信号
    void updateTree(int row, QString text2);
    void writeMessage(int i,QString text);
    void changestatus(int column,QString text);
    void startConnect();

};

#endif // CAMERA_H

