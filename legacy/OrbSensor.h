#include "libobsensor/hpp/Pipeline.hpp"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>

class OrbSensor
{
private:
    const int WIDTH = 640;
    const int HEIGHT = 480;
    const int wait_time = 2000;
    std::mutex                 videoFrameMutex;
    std::shared_ptr<ob::VideoFrame> colorFrame;
    std::shared_ptr<ob::VideoFrame> depthFrame;

    std::vector<std::shared_ptr<ob::VideoFrame>> frames;
    ob::Pipeline pipe;
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

public:
    enum CameraType {
        ColorCamera=0,
        DepthCamera=1,
        AllCamera=2,
    };
    
    OrbSensor(/* args */);
    ~OrbSensor();
    std::tuple<cv::Mat, cv::Mat> connect(CameraType type);
    void disconnect();
    std::tuple<cv::Mat, cv::Mat> getVideoFrame();
    cv::Mat getColorFrame();
    cv::Mat getDepthFrame();
    cv::Mat frame2mat(const std::shared_ptr<ob::VideoFrame> &frame);
    
};


