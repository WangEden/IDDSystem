#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <opencv2/core/matx.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include "types.h"


std::tuple<float, float, float> leastSquaresFit(const std::vector<int>& x_values, const std::vector<int>& y_values);
/**
 * @brief 去除贴纸
 * 
 * @param bboxes     Yolov8识别到的所有物品的bbox
 * @param matrix    相机内参矩阵
 * @param depth     当前帧的深度图
 */
void removeTheSticker(std::vector<std::vector<Detection> >& yoloRectses, std::vector<cv::Mat> & depth, float threshold);

/**
 * @brief 根据bbox将深度图部分区域转换为点云
 * 
 * @param matrix    相机内参矩阵
 * @param bbox      感兴趣区域
 * @param depth     当前帧的深度图
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr depth2cloudByBBox(cv::Mat matrix, cv::Rect bbox, cv::Mat depth);

/**
 * @brief 将Yolo模型输出的框转换为OpenCV的Rect
 * 
 * @param yoloDects 
 * @param width 
 * @param height 
 * @return cv::Rect 
 */
// cv::Rect yoloRect2bboxRect(cv::Vec4f yoloDects);

/**
 * @brief 判断该部分点云是否是贴纸
 * 
 * @param inputCloud 
 * @return float 
 */
float isSticker(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
