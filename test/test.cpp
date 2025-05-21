#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "创建相机变量" << std::endl;
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    std::cout << "创建相机变量完成" << std::endl;
    if (!cap.isOpened()) {
        std::cerr << "无法打开相机！" << std::endl;
        return -1;
    }
    cv::Mat frame;
    std::cout << "相机已打开..." << std::endl;

    while (true) {
        std::cout << "读取中..." << std::endl;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法捕获视频帧！" << std::endl;
            break;
        }
        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    cap.release();
    cv::destroyAllWindows();

    return 0;
}

