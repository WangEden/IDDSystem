#include "planeDetectionPCL.h"


std::tuple<float, float, float> leastSquaresFit(const std::vector<int>& x_values, const std::vector<int>& y_values)
{
    assert(x_values.size() == y_values.size());

    // 将std::vector转换为Eigen::Vector
    Eigen::VectorXf x(x_values.size());
    Eigen::VectorXf y(y_values.size());
    for (size_t i = 0; i < x_values.size(); ++i)
    {
        x(i) = x_values[i];
        y(i) = y_values[i];
    }

    // 计算均值
    float mean_x = x.mean();
    float mean_y = y.mean();

    // 计算斜率
    float slope = (x.array() - mean_x).cwiseProduct(y.array() - mean_y).sum() / (x.array() - mean_x).square().sum();

    // 计算截距
    float intercept = mean_y - slope * mean_x;

    // 计算相关系数
    float stddev_x = std::sqrt((x.array() - mean_x).square().sum() / x.size());
    float stddev_y = std::sqrt((y.array() - mean_y).square().sum() / y.size());
    float covariance = (x.array() - mean_x).cwiseProduct(y.array() - mean_y).sum() / x.size();
    float correlation_coefficient = covariance / (stddev_x * stddev_y);

    return std::make_tuple(slope, intercept, correlation_coefficient);
}


void removeTheSticker(std::vector<std::vector<Detection> >& res_batch, std::vector<cv::Mat> & depthImg, float threshold)
{
try {
        // if (res_batch.size() != 0 && res_batch[0].size() != 0 && !depthImg[0].empty()) {
        //     // std::cout << "depth: " << std::endl;
        //     // std::cout << depthImg[0] << std::endl;
        //     // 依次处理每个bbox
        //     for (size_t i = 0; i < res_batch[0].size(); i++)
        //     {
        //         // std::cout << "res_batch[0]: " << res_batch[0][i].bbox[0] << " " <<  res_batch[0][i].bbox[1] << " " << res_batch[0][i].bbox[2] << " " << res_batch[0][i].bbox[3] << std::endl;
        //         cv::Vec4f yoloRect(res_batch[0][i].bbox[0], res_batch[0][i].bbox[1], res_batch[0][i].bbox[2], res_batch[0][i].bbox[3]);
        //         cv::Rect bbox = yoloRect2bboxRect(yoloRect);
        //         // std::cout << "bbox: " << bbox.x << " " <<  bbox.y << " " << bbox.width << " " << bbox.height << std::endl;

        //         if ( (bbox.x < 0 || bbox.x > 640) && (bbox.y < 0 || bbox.y > 480) &&
        //             (bbox.x + bbox.width > 640) && (bbox.y + bbox.height > 480))
        //             continue;

        //         int center_x = bbox.x + bbox.width / 2;
        //         // 在bbox中沿着中线从下往上将深度值取出之后画在折线图上
        //         std::vector<float> depth_values;
        //         for (int j = bbox.y + bbox.height - 1; j >= bbox.y; j--)
        //         {
        //             depth_values.push_back(depthImg[0].at<uint16_t>(j, center_x));
        //         }
        //         // 画折线图
        //         // cv::Mat plot = cv::Mat::zeros(480, 640, CV_8UC3);
        //         int x_offset = 10;
        //         int x_begin = 5;
        //         std::vector<int> x_values;
        //         std::vector<int> y_values;
        //         float max_depth = *std::max_element(depth_values.begin(), depth_values.end());
        //         float min_depth = *std::min_element(depth_values.begin(), depth_values.end());
        //         // 归一化深度值
        //         for (int j = 0; j < depth_values.size(); j++)
        //         {
        //             x_values.push_back(x_begin + j * x_offset);
        //             int y = (depth_values[j] - min_depth) / (max_depth - min_depth) * 100;
        //             y_values.push_back(y);
        //         }
        //         // 最小二乘法拟合两个点集，计算斜率和相关系数
        //         auto data_tuple = leastSquaresFit(x_values, y_values);
        //         float slope = std::get<0>(data_tuple);
        //         float intercept = std::get<1>(data_tuple);
        //         float correlation_coefficient = std::get<2>(data_tuple);
        //         std::cout << res_batch[0][i].class_id << " coefficient: " << correlation_coefficient << std::endl;
        //         // 相关系数大于0.98的认为是贴纸
        //         if (correlation_coefficient > threshold)
        //         {
        //             res_batch[0].erase(res_batch[0].begin() + i);
        //         }
        //     }
        // }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

// cv::Rect yoloRect2bboxRect(cv::Vec4f yoloRect)
// {
//     // yoloRect: [class_center_x_norm, class_center_y_norm, w_norm, h_norm]
//     // bbox: [x, y, w, h]
//     cv::Rect bbox;
//     bbox.width = static_cast<int>(yoloRect[2] - yoloRect[0]);
//     bbox.height = static_cast<int>(yoloRect[3] - yoloRect[1]);
//     bbox.x = static_cast<int>(yoloRect[0]);
//     bbox.y = static_cast<int>(yoloRect[1]);
//     return bbox;
// }

// 判断是不是贴纸
float isSticker(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
    // 计算点云两点间的最小距离
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(inputCloud);

    float min_distance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < inputCloud->points.size(); ++i) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(inputCloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (size_t j = 1; j < pointIdxNKNSearch.size(); ++j) { // start from 1 to skip the point itself
                min_distance = std::min(min_distance, std::sqrt(pointNKNSquaredDistance[j]));
            }
        }
    }

    // std::cout << ", min distance: " << min_distance;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选设置
    seg.setOptimizeCoefficients (true);
    // 必要的设置
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    seg.setDistanceThreshold (static_cast<int>(min_distance * 10)); // 根据点云间最小距离设置距离阈值

    seg.setInputCloud (inputCloud);
    seg.segment (*inliers, *coefficients);

    float rate = inliers->indices.size () * 1.0 / inputCloud->points.size() * 1.0;
    // std::cout << ", plane proportion: " << rate;

    return rate; // 返回点云在平面模型上的分布比例
}

// 根据bbox得到深度图中的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr depth2cloudByBBox(
    cv::Mat matrix, cv::Rect bbox, cv::Mat depth
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double fx = matrix.at<double>(0, 0);
    double fy = matrix.at<double>(1, 1);
    double cx = matrix.at<double>(0, 2);
    double cy = matrix.at<double>(1, 2);

    for (int i = bbox.y; i < bbox.y + bbox.height; i++) {
        for (int j = bbox.x; j < bbox.x + bbox.width; j++) {
            // 获取深度图中(m,n)处的值
            short d = depth.ptr<short>(i)[j];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            pcl::PointXYZ p;

            // 计算这个点的空间坐标
            p.z = double(d) / -1;
            p.x = (cx - j) * p.z / fx;
            p.y = (i - cy) * p.z / fy;

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}