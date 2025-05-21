#pragma execution_character_set("utf-8")
#include "camera.h"
#include "yolov8_det.h"
#include <QDebug>
#include <QThread>
#include <numeric>
#include <chrono>

const int MIN = 20;

Camera::Camera(QObject *parent) : QObject(parent)
{

}
void Camera::PreStart()
{
    
    
}
void Camera::openCamera()
{
    emit changestatus(1,"识别中");
    emit startConnect();
    this->cap = new VideoCapture();
    this->cap->open(0, cv::CAP_V4L2);

    this->cameraIsOpened = true;
    emit this->cameraIsOpen(true);     // 相机打开

    this->capTimer = new QTimer;
    this->yoloTimer = new QTimer;

    connect(this->capTimer, &QTimer::timeout, [=](){
        // 采集图像
        std::vector<Mat> frame(1); 
        this->frame_lock.lock();
        this->cap->read(frame[0]);
        yolov8_work(frame);
        for(int i = 0;i<21;i++)
        {
            int num = 0;
            for (int j = 0; j < 50; j++) {
                if (num_cnt[i][j] > MIN && num_cnt[i][j] > num_cnt[i][num]) {  // 找到超过阈值且出现次数最多的数量
                    num = j;
                }
            }
            emit updateTree(i,QString::number(num));
        }
        for(int i = 0;i<21;i++)
        {
            emit updateTree(i,QString::number(Max[i]));
        }
        // 将OpenCv的BGR图像转换成正常的RGB图像
        cvtColor(frame[0], frame[0], cv::COLOR_BGR2RGB);
        // 将OpenCv的图像转换成Qt的QImage
        QPixmap showImage = QPixmap::fromImage(QImage((const uchar*)(frame[0].data),
                                                    frame[0].cols,
                                                    frame[0].rows,
                                                    frame[0].step,
                                                    QImage::Format_RGB888));
        // 将QPixmap图像通过信号发送出去
        emit this->cameraShowImage(showImage);
        this->frame_lock.unlock();
    });

    connect(this->yoloTimer, &QTimer::timeout, [=]() {
        this->closeCamera();
    });
    // 采样时间可以根据相机的FPS修改
    this->capTimer->start(25);
    yoloTimer->start(20000);
}

void Camera::closeCamera()
{
    yoloTimer->stop();
    yoloTimer->deleteLater();
    //emit write signal
    emit changestatus(1,"结束");
    emit writeMessage(-1,"START");
    for(int i = 0;i<21;i++)
    {
        if(Max[i] > 0)
        emit writeMessage(i,QString::number(Max[i]));
    }
    emit writeMessage(-2,"END");
}

void Camera::Camerastop()
{

    yolov8_stop();
    qDebug() << "子线程：" << QThread::currentThreadId();
    if(!this->cameraIsOpened) return;

    if(this->capTimer->isActive()){
        this->capTimer->stop();
        this->capTimer->deleteLater();

        this->cameraIsOpened = false;
        emit this->cameraIsOpen(false);     // 相机关闭
        cv::destroyAllWindows();	// 销毁OpenCv的窗口
        // this->cap->release();		// 释放相机
        //     if (this->color_camera != nullptr)
        //         this->color_camera->disconnect();
        //     if (this->depth_camera != nullptr)
        //         this->depth_camera->disconnect();
    }
}

cv::Rect Camera::MoveDetect(cv::Mat temp, cv::Mat frame)  
{  
    //1.将background和frame转为灰度图  
    cv::Mat gray1, gray2;  
    cv::cvtColor(temp, gray1, cv::COLOR_BGR2GRAY);  
    cv::cvtColor(frame, gray2, cv::COLOR_BGR2GRAY);  
    //2.将background和frame做差  
    cv::Mat diff;  
    cv::absdiff(gray1, gray2, diff);  
    // imshow("diff", diff);  

    //3.对差值图diff_thresh进行阈值化处理
    cv::Mat diff_thresh;
    cv::threshold(diff, diff_thresh, 60, 255, THRESH_BINARY);
    // imshow("diff_thresh", diff_thresh);
 
    //6.查找轮廓并绘制轮廓  
    std::vector<std::vector<Point> > contours;  
    cv::findContours(diff_thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //7.查找正外接矩形  
    std::vector<Rect> boundRect(contours.size());
    for (int i = 0; i < contours.size(); i++)  
    {  
        boundRect[i] = cv::boundingRect(contours[i]);  
        // rectangle(result, boundRect[i], Scalar(0, 255, 0), 2);//在result上绘制正外接矩形  
    }
    // 遍历矩形序列，找到所有矩形的最小外接矩形
    if (contours.size() > 0) {
        cv::Rect minRect = cv::boundingRect(contours[0]);
        for (int i = 1; i < contours.size(); i++)
        {
            minRect = minRect | cv::boundingRect(contours[i]);
        }
        return minRect;
    }
    else {
        return cv::Rect(320, 240, 0, 0);
    }
    
    // rectangle(result, minRect, Scalar(255, 0, 0), 2);//在result上绘制正外接矩形
    // return result;//返回result  
}