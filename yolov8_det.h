#ifndef YOLOV8_DET_H
#define YOLOV8_DET_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "postprocess.h"
#include <QObject>

void yolov8_start();
// void yolov8_work(std::vector<cv::Mat> &img, std::vector<cv::Mat> &depthImg, cv::Rect roi, cv::Mat matrix);
void yolov8_work(std::vector<cv::Mat> &img);
void yolov8_stop();



#endif




