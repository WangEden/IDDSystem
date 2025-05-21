#include "OrbSensor.h"
#include <tuple>


OrbSensor::OrbSensor()
{
}

OrbSensor::~OrbSensor()
{
}

std::tuple<cv::Mat, cv::Mat> OrbSensor::connect(CameraType type)
{
    try {
        switch (type)
        {
        case ColorCamera: // 彩色
        {
            auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
            auto colorProfile  = colorProfiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_RGB, 30);
            config->enableStream(colorProfile->as<ob::VideoStreamProfile>());
        }break;
        case DepthCamera: // 深度
        {
            auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
            auto depthProfile = depthProfiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);
            config->enableStream(depthProfile->as<ob::VideoStreamProfile>());
        }break;
        case AllCamera:
        {
            auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
            auto colorProfile  = colorProfiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_RGB, 30);
            config->enableStream(colorProfile->as<ob::VideoStreamProfile>());
            auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
            auto depthProfile = depthProfiles->getVideoStreamProfile(640, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);
            config->enableStream(depthProfile->as<ob::VideoStreamProfile>());
        }break;
        default:break;
        }
    }
    catch(...) {
        switch (type)
        {
        case ColorCamera:
            std::cout << "color stream not found!" << std::endl;
        break;
        case DepthCamera:
            std::cout << "depth stream not found!" << std::endl;
        break;
        }
    }
    // try {
    //     // 根据指定的格式查找对应的Profile,优先查找Y16格式
    //     depthProfile  = depthProfiles->getVideoStreamProfile(this->WIDTH, OB_HEIGHT_ANY, OB_FORMAT_Y16, 30);
    // } catch (const ob::Error &) {
    //     // 没找到Y16格式后不匹配格式查找对应的Profile进行开流
    //     depthProfile  = depthProfiles->getVideoStreamProfile(this->WIDTH, OB_HEIGHT_ANY, OB_FORMAT_UNKNOWN, 30);
    // }
    // config->enableStream(depthProfile);
    // }
    // catch(...) {
    // }
    try {
        const auto &device = pipe.getDevice();
        if (device->isPropertySupported(OB_PROP_DEPTH_MIRROR_BOOL, OB_PERMISSION_WRITE)) {
            device->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);
        }
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Current device is not support depth mirror!" << std::endl;
    }
    // auto irProfiles = pipe.getStreamProfileList(OB_SENSOR_IR); 红外
    // auto irProfile  = irProfiles->getProfile(OB_PROFILE_DEFAULT);
    // config->enableStream(irProfile->as<ob::VideoStreamProfile>());
    try {
        config->setAlignMode(ALIGN_D2C_HW_MODE);
        switch (type)
        {
        case ColorCamera:
        case DepthCamera:
        {
            this->pipe.start(config);
        }break;
        case AllCamera:
        {
            pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameset) {
                std::unique_lock<std::mutex> lk(videoFrameMutex);
                // this->colorFrame = dynamic_cast<ob::VideoFrame*>(frameset->colorFrame());
                this->colorFrame = (frameset->colorFrame());
                // this->depthFrame = dynamic_cast<ob::VideoFrame*>(frameset->depthFrame());
                this->depthFrame = (frameset->depthFrame());
            });
        }break;
        }
        // pipe.start(config, [&](std::shared_ptr<ob::FrameSet> frameset) {
        //     std::unique_lock<std::mutex> lk(videoFrameMutex);
        //     colorFrame = frameset->colorFrame();
        //     depthFrame = frameset->depthFrame();
        //     irFrame    = frameset->irFrame();
        // });
        // 获取深度相机内参
        switch (type)
        {
        case ColorCamera: // 彩色
        {
            return std::make_tuple(cv::Mat(), cv::Mat());            
        }break;
        case DepthCamera: // 深度
        case AllCamera:
        {
            OBCameraParam cameraParam = pipe.getCameraParam();
            auto depIntr = cameraParam.depthIntrinsic;
            OBCameraDistortion depDist = cameraParam.depthDistortion;
            double k1 = depDist.k1;
            double k2 = depDist.k2;
            double p1 = depDist.p1;
            double p2 = depDist.p2;
            double k3 = depDist.k3;
            cv::Mat depIntrinsic = (cv::Mat_<double>(3, 3) << \
            depIntr.fx, 0.0, depIntr.cx, \
            0.0, depIntr.fy, depIntr.cy, \
            0.0, 0.0, 1.0);
            cv::Mat depDistortion = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
            return std::make_tuple(depIntrinsic.clone(), depDistortion.clone());        
        }break;
        default:break;
        }
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Current device is not support depth sensor!" << std::endl;
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
    catch (...)
    {
        std::cerr << "Unknown error" << '\n';
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
}

void OrbSensor::disconnect()
{
    this->pipe.stop();
}

std::tuple<cv::Mat, cv::Mat> OrbSensor::getVideoFrame()
{
    try
    {
        // auto frameSet = pipe.waitForFrames(this->wait_time);
        // while (frameSet == nullptr) {
        //     frameSet = pipe.waitForFrames(this->wait_time);
        // }
        // auto colorFrame = frameSet->colorFrame();
        // cv::Mat colorMat = frame2mat(colorFrame);
        // return colorMat.clone();

        auto colorFrame = frame2mat(this->colorFrame);
        auto depthFrame = frame2mat(this->depthFrame);

        return std::make_tuple(colorFrame.clone(), depthFrame.clone());
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Current device is not support color sensor!" << std::endl;
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
    catch (...)
    {
        std::cerr << "Unknown error" << '\n';
        return std::make_tuple(cv::Mat(), cv::Mat());
    }
}

cv::Mat OrbSensor::getColorFrame()
{
    try
    {
        auto frameSet = pipe.waitForFrames(this->wait_time);
        while (frameSet == nullptr) {
            frameSet = pipe.waitForFrames(this->wait_time);
        }
        auto colorFrame = frameSet->colorFrame();
        cv::Mat colorMat = frame2mat(colorFrame);
        return colorMat.clone();
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Current device is not support color sensor!" << std::endl;
        return cv::Mat();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return cv::Mat();
    }
    catch (...)
    {
        std::cerr << "Unknown error" << '\n';
        return cv::Mat();
    }
}

cv::Mat OrbSensor::getDepthFrame()
{
    try
    {
        auto frameSet = pipe.waitForFrames(this->wait_time);
        while (frameSet == nullptr) {
            frameSet = pipe.waitForFrames(this->wait_time);
        }
        auto depthFrame = frameSet->depthFrame();
        cv::Mat depthMat = frame2mat(depthFrame);
        return depthMat.clone();
    }
    catch (const ob::Error &e)
    {
        std::cerr << "Current device is not support depth sensor!" << std::endl;
        return cv::Mat();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return cv::Mat();
    }
    catch (...)
    {
        std::cerr << "Unknown error" << '\n';
        return cv::Mat();
    }
}

cv::Mat OrbSensor::frame2mat(const std::shared_ptr<ob::VideoFrame> &frame)
{
    // const int data_size = static_cast<int>(frame->dataSize());
    if (frame == nullptr) 
    {   
        // std::cout << "frame2mat: frame empty" << std::endl;
        return {};
    }
    const int data_size = static_cast<int>(frame->dataSize());
    if (data_size < 1024) {
        return {};
    }

    const OBFrameType frame_type = frame->type();               // 帧类型（彩色/深度/红外）
    const OBFormat frame_format = frame->format();              // 图像格式
    const int frame_height = static_cast<int>(frame->height()); // 图像高度
    const int frame_width = static_cast<int>(frame->width());   // 图像宽度
    void *const frame_data = frame->data();                     // 帧原始数据

    cv::Mat result_mat;
    if (frame_type == OB_FRAME_COLOR) {
        // Color image
        if (frame_format == OB_FORMAT_MJPG) {
            const cv::Mat raw_mat(1, data_size, CV_8UC1, frame_data);
            result_mat = cv::imdecode(raw_mat, 1);
        } else if (frame_format == OB_FORMAT_NV21) {
            const cv::Mat raw_mat(frame_height * 3 / 2, frame_width, CV_8UC1, frame_data);
            cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_NV21);
        } else if (frame_format == OB_FORMAT_YUYV || frame_format == OB_FORMAT_YUY2) {
            const cv::Mat raw_mat(frame_height, frame_width, CV_8UC2, frame_data);
            cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_YUY2);
        } else if (frame_format == OB_FORMAT_RGB888) {
            const cv::Mat raw_mat(frame_height, frame_width, CV_8UC3, frame_data);
            cv::cvtColor(raw_mat, result_mat, cv::COLOR_RGB2BGR);
        } else if (frame_format == OB_FORMAT_UYVY) {
            const cv::Mat raw_mat(frame_height, frame_width, CV_8UC2, frame_data);
            cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_UYVY);
        }
    } else if (frame_format == OB_FORMAT_Y16 || frame_format == OB_FORMAT_YUYV ||
               frame_format == OB_FORMAT_YUY2) {
        // IR or depth image
        const cv::Mat raw_mat(frame_height, frame_width, CV_16UC1, frame_data);
        // const double scale = 1 / pow(2, frame->pixelAvailableBitSize() -
        //                                     (frame_type == OB_FRAME_DEPTH ? 10 : 8));
        // cv::convertScaleAbs(raw_mat, result_mat, scale);
        result_mat = raw_mat.clone();
    } else if (frame_type == OB_FRAME_IR) {
        // IR image
        if (frame_format == OB_FORMAT_Y8) {
            result_mat = cv::Mat(frame_height, frame_width, CV_8UC1, frame_data);
        } else if (frame_format == OB_FORMAT_MJPG) {
            const cv::Mat raw_mat(1, data_size, CV_8UC1, frame_data);
            result_mat = cv::imdecode(raw_mat, 1);
        }
    }
    return result_mat;
}