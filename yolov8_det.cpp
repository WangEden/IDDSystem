#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cuda_utils.h"
#include "logging.h"
#include "model.h"

#include "preprocess.h"
#include "utils.h"
#include "yolov8_det.h"

// #include "planeDetectionPCL.h"
#include <cmath>
#include <QDebug>

Logger gLogger;
using namespace nvinfer1;
const int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

// std::map<int, std::string> ClassesName = {
//     {0, "CA001"}, {1, "CA002"}, {2, "CA003"}, {3, "CA004"}, 
//     {4, "CB001"}, {5, "CB002"}, {6, "CB003"}, {7, "CB004"},
//     {8, "CC001"}, {9, "CC002"}, {10, "CC003"}, {11, "CC004"},
//     {12, "CD001"}, {13, "CD002"}, {14, "CD003"}, {15, "CD004"},
//     {16, "TABLE"},
//     {17, "W001"}, {18, "W002"}, {19, "W003"}, {20, "W004"},
// };
std::map<int, std::string> ClassesName = {
    {0, "勺子"}, {1, "筷子"}, {2, "碗"}, {3, "衣架"}, 
    {4, "沙琪玛"}, {5, "罐装蜜饯"}, {6, "火腿肠"}, {7, "薯片"},
    {8, "罐装饮料"}, {9, "瓶装饮料"}, {10, "盒装牛奶"}, {11, "瓶装饮用水"},
    {12, "苹果"}, {13, "橙子"}, {14, "香蕉"}, {15, "芒果"},
    {16, "桌子"},
    {17, "W001"}, {18, "W002"}, {19, "W003"}, {20, "W004"},
};

void serialize_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                      float& gw, int& max_channels) {
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();
    IHostMemory* serialized_engine = nullptr;

    if (is_p == 6) {
        serialized_engine = buildEngineYolov8DetP6(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else if (is_p == 2) {
        serialized_engine = buildEngineYolov8DetP2(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    } else {
        serialized_engine = buildEngineYolov8Det(builder, config, DataType::kFLOAT, wts_name, gd, gw, max_channels);
    }

    assert(serialized_engine);
    std::ofstream p(engine_name, std::ios::binary);
    if (!p) {
        std::cout << "could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

    delete serialized_engine;
    delete config;
    delete builder;
}

void deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine,
                        IExecutionContext** context) {
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
}

void prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                    float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                    std::string cuda_post_process) {
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
    if (cuda_post_process == "c") {
        *output_buffer_host = new float[kBatchSize * kOutputSize];
    } else if (cuda_post_process == "g") {
        if (kBatchSize > 1) {
            std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
            exit(0);
        }
        // Allocate memory for decode_ptr_host and copy to device
        *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
        CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
    }
}

void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
           float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process) {
    // infer on the batch asynchronously, and DMA output back to host
    auto start = std::chrono::system_clock::now();
    context.enqueue(batchsize, buffers, stream, nullptr);
    if (cuda_post_process == "c") {
        CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost,
                                   stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } else if (cuda_post_process == "g") {
        CUDA_CHECK(
                cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
        cuda_decode((float*)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
        cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);  //cuda nms
        CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                                   sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost,
                                   stream));
        auto end = std::chrono::system_clock::now();
        std::cout << "inference and gpu postprocess time: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    CUDA_CHECK(cudaStreamSynchronize(stream));
}

bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, int& is_p, std::string& img_dir,
                std::string& sub_type, std::string& cuda_post_process, float& gd, float& gw, int& max_channels) {
    if (argc < 4)
        return false;
    if (std::string(argv[1]) == "-s" && (argc == 5 || argc == 7)) {
        wts = std::string(argv[2]);
        engine = std::string(argv[3]);
        auto sub_type = std::string(argv[4]);

        if (sub_type[0] == 'n') {
            gd = 0.33;
            gw = 0.25;
            max_channels = 1024;
        } else if (sub_type[0] == 's') {
            gd = 0.33;
            gw = 0.50;
            max_channels = 1024;
        } else if (sub_type[0] == 'm') {
            gd = 0.67;
            gw = 0.75;
            max_channels = 576;
        } else if (sub_type[0] == 'l') {
            gd = 1.0;
            gw = 1.0;
            max_channels = 512;
        } else if (sub_type[0] == 'x') {
            gd = 1.0;
            gw = 1.25;
            max_channels = 640;
        } else {
            return false;
        }
        if (sub_type.size() == 2 && sub_type[1] == '6') {
            is_p = 6;
        } else if (sub_type.size() == 2 && sub_type[1] == '2') {
            is_p = 2;
        }
    } else if (std::string(argv[1]) == "-d" && argc == 5) {
        engine = std::string(argv[2]);
        img_dir = std::string(argv[3]);
        cuda_post_process = std::string(argv[4]);
    } else {
        return false;
    }
    return true;
}

IRuntime* runtime = nullptr;
ICudaEngine* engine = nullptr;
IExecutionContext* context = nullptr;
int model_bboxes;
cudaStream_t stream;
float* device_buffers[2];
float* output_buffer_host = nullptr;
float* decode_ptr_host = nullptr;
float* decode_ptr_device = nullptr;
std::string wts_name = "";
std::string engine_name = "../engines/IDD.engine";
std::string img_dir = ".";
std::string sub_type = "";
std::string cuda_post_process = "g";
int status; // 0 - stop, 1 - running

void yolov8_start() {
    std::cout << "start yolo" << std::endl;
    if (status == 1) {
        std::cerr << "yolov8 start fail: yolov8 is running now..." << std::endl;
        return;
    }
    status = 1;
    cudaSetDevice(kGpuId);
    int is_p = 0;
    float gd = 0.0f, gw = 0.0f;
    int max_channels = 0;

    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
        serialize_engine(wts_name, engine_name, is_p, sub_type, gd, gw, max_channels);
        return;
    }

    // Deserialize the engine from file
    deserialize_engine(engine_name, &runtime, &engine, &context);
    // cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    cuda_preprocess_init(kMaxInputImageSize);
    auto out_dims = engine->getBindingDimensions(1);
    model_bboxes = out_dims.d[0];

    prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
                   &decode_ptr_device, cuda_post_process);

    qDebug() << "yolo start ok";
}

void yolov8_work(std::vector<cv::Mat> &img) {
    if (status != 1) {
        std::cerr << "yolov8 work fail: yolov8 is not running..." << std::endl;
        return;
    }

    // Preprocess
    cuda_batch_preprocess(img, device_buffers[0], kInputW, kInputH, stream);
    // Run inference
    infer(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host,
            decode_ptr_device, model_bboxes, cuda_post_process);
    std::vector<std::vector<Detection>> res_batch;
    if (cuda_post_process == "c") {
        // NMS
        (res_batch, output_buffer_host, 1, kOutputSize, kConfThresh, kNmsThresh);
    } else if (cuda_post_process == "g") {
        //Process gpu decode and nms results
        batch_process(res_batch, decode_ptr_host, 1, bbox_element, img);
    }
    
    // *******************************************************************************
    // // remove the sticker first
    // double threshold = 0.97;
    // double threshold_slope = 0.9;
    // int x_offset = 10;
    // int x_begin = 5;
    // int center_x, y;
    // double max_depth, min_depth;
    // std::vector<double> depth_values;
    // std::vector<int> x_values;
    // std::vector<int> y_values;
    // // cv::Rect bbox;
    // cv::Vec4f yoloRect;
    // std::vector<Detection> res_batch_1;
    // std::vector<Detection> res_batch_2;
    // // for (auto det : res_batch_1) {
    // //     cout << "det: " << det.bbox[0] << " " << det.bbox[1] << " " << det.bbox[2] << " " << det.bbox[3] << std::endl;
    // // }
    // // std::cout << "before size: " << res_batch.size();
    // // **************************************************************
    // // remove the item out of roi
    // // std::cout << "before reduce size:" << res_batch_1.size() << std::endl;
    // // std::cout << "roi: " << roi.x << " " << roi.y << " " << roi.width << " " << roi.height << std::endl;
    // // 椭圆方程：x^2 / a^2 + y^2 / b^2 = 1
    // double a = roi.width / 2.0;
    // double b = roi.height / 2.0;
    // double tcirclex = roi.x + a;
    // double tcircley = roi.y + b;
    // a = std::pow(a, 2);
    // b = std::pow(b, 2);
    // double itemCenterX, itemCenterY;
    // if (res_batch.size() != 0) {
    //     for (size_t j = 0; j < res_batch[0].size(); j++)
    //     {
    //         cv::Rect bbox = get_rect(img[0], res_batch[0][j].bbox);
    //         itemCenterX = bbox.x + bbox.width / 2.0;
    //         itemCenterY = bbox.y + bbox.height / 2.0;
    //         if ( std::pow((itemCenterX - tcirclex), 2) / a + std::pow((itemCenterY - tcircley), 2) / b < 1) {
    //             res_batch_1.push_back(res_batch[0][j]);
    //             // std::cout << "out of area removed" << std::endl;
    //         }
    //         // if ((res_batch_1[j].bbox[0] > roi.x + roi.width) || // 在框右边
    //         //     (res_batch_1[j].bbox[1] > roi.y + roi.height) || // 在框下边
    //         //     (res_batch_1[j].bbox[2] < roi.x) || // 在框左边
    //         //     (res_batch_1[j].bbox[3] < roi.y) // 在框上班
    //         // )
    //     }
    //     res_batch.clear();
    // }
    // // **************************************************************
    // // std::cout << "depth: " << std::endl;
    // // std::cout << depthImg[0] << std::endl;
    // // **************************************************************
    // try {
    //     std::cout<<"start------------------------------------------------"<<std::endl;
    //     // std::vector<bool> isremove;
    //     // for (size_t i = 0; i < res_batch_1.size(); i++)
    //     // {
    //     //     isremove.push_back(false);
    //     // }
    //     if (res_batch_1.size() != 0 && !depthImg[0].empty()) {
    //         for (size_t i = 0; i < res_batch_1.size(); i++)
    //         {
    //             // yoloRect[0] = res_batch_1[i].bbox[0];
    //             // yoloRect[1] = res_batch_1[i].bbox[1];
    //             // yoloRect[2] = res_batch_1[i].bbox[2];
    //             // yoloRect[3] = res_batch_1[i].bbox[3];
    //             // bbox.width = static_cast<int>(yoloRect[2] - yoloRect[0]);
    //             // bbox.height = static_cast<int>(yoloRect[3] - yoloRect[1]);
    //             // bbox.x = static_cast<int>(yoloRect[0]);
    //             // bbox.y = static_cast<int>(yoloRect[1]);
    //             cv::Rect bbox = get_rect(img[0], res_batch_1[i].bbox);

    //             // cout << "resbox: " << bbox.x << " " << bbox.y << " " << bbox.width << " " << bbox.height;
    //             cout << "ClassID: " << ClassesName[int(res_batch_1[i].class_id)];

    //             if ( (bbox.x < 0 || bbox.x > 640) || (bbox.y < 0 || bbox.y > 480) ||
    //                 (bbox.x + bbox.width > 640) || (bbox.y + bbox.height > 480))
    //                 continue;
                
    //             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = depth2cloudByBBox(matrix, bbox, depthImg[0]);

    //             // 判断点云是否为空
    //             if (cloud->points.size() == 0)
    //             {
    //                 cout << "bbox: " << bbox << endl;
    //                 cout << "the box cloud is empty" << endl;
    //                 continue;
    //             }

    //             // std::cout << res_batch_1[i].class_id;
    //             float result = isSticker(cloud);
    //             // if (correlation_coefficient > threshold && slope > threshold_slope)
    //             std::cout << " rate: " << result;
    //             if (result > threshold)
    //             {
    //                 std::cout << " removed" << std::endl;
    //                 // std::cout << "erase before: " << res_batch_1.size();
    //                 // res_batch_1.erase(res_batch_1.begin() + i);
    //                 // isremove[i] = true;
    //                 // std::cout << " erase after: " << res_batch_1.size() << std::endl;
    //             }
    //             else {
    //                 res_batch_2.push_back(res_batch_1[i]);
    //                 std::cout << " cao" << std::endl;
    //             }
    //         }
    //         // for (size_t i = 0; i < isremove.size(); i++) {
    //         //     if (!isremove[i])
    //         //         res_batch_2.push_back(res_batch_1[i]);
    //         // }
    //     }
    //     else {
    //         std::cout << "depth empty!" << std::endl;
    //     }
        
    // }
    // catch (const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
    // std::cout << "end res_batch_1_size: " << res_batch_1.size() << std::endl;
    // std::cout << "res_batch_1 size: "<< res_batch_1.size()<<std::endl;
    // res_batch.push_back(res_batch_2);
    /*
    try {
        // if (res_batch.size() != 0 && res_batch[0].size() != 0 && !depthImg[0].empty()) {
            // std::cout << "depth: " << std::endl;
            // std::cout << depthImg[0] << std::endl;
            // 依次处理每个bbox
            for (size_t i = 0; i < res_batch_1.size(); i++)
            {
                depth_values.clear();
                x_values.clear();
                y_values.clear();
                // std::cout << "res_batch_1: " << res_batch_1[i].bbox[0] << " " <<  res_batch_1[i].bbox[1] << " " << res_batch_1[i].bbox[2] << " " << res_batch_1[i].bbox[3] << std::endl;
                yoloRect[0] = res_batch_1[i].bbox[0];
                yoloRect[1] = res_batch_1[i].bbox[1];
                yoloRect[2] = res_batch_1[i].bbox[2];
                yoloRect[3] = res_batch_1[i].bbox[3];
                bbox.width = static_cast<int>(yoloRect[2] - yoloRect[0]);
                bbox.height = static_cast<int>(yoloRect[3] - yoloRect[1]);
                bbox.x = static_cast<int>(yoloRect[0]);
                bbox.y = static_cast<int>(yoloRect[1]);
                // bbox = yoloRect2bboxRect(yoloRect);
                // std::cout << "bbox: " << bbox.x << " " <<  bbox.y << " " << bbox.width << " " << bbox.height << std::endl;

                if ( (bbox.x < 0 || bbox.x > 640) || (bbox.y < 0 || bbox.y > 480) ||
                    (bbox.x + bbox.width > 640) || (bbox.y + bbox.height > 480))
                    continue;

                // center_x = bbox.x + bbox.width / 4 * 3;
                center_x = bbox.x + bbox.width / 2;
                // 在bbox中沿着中线从下往上将深度值取出之后画在折线图上
                for (int j = bbox.y + bbox.height - 1; j >= bbox.y; j--)
                {
                    depth_values.push_back(depthImg[0].at<uint16_t>(j, center_x));
                }
                // 画折线图
                // cv::Mat plot = cv::Mat::zeros(480, 640, CV_8UC3);
                max_depth = *std::max_element(depth_values.begin(), depth_values.end());
                min_depth = *std::min_element(depth_values.begin(), depth_values.end());
                // 归一化深度值
                for (int j = 0; j < depth_values.size(); j++)
                {
                    x_values.push_back(x_begin + j * x_offset);
                    y = (depth_values[j] - min_depth) / (max_depth - min_depth) * 100;
                    y_values.push_back(y);
                }
                // 最小二乘法拟合两个点集，计算斜率和相关系数
                auto data_tuple = leastSquaresFit(x_values, y_values);
                double slope = std::get<0>(data_tuple);
                double intercept = std::get<1>(data_tuple);
                double correlation_coefficient = std::get<2>(data_tuple);
                std::cout << res_batch_1[i].class_id << " coefficient: " << correlation_coefficient << " slope: " << slope;
                // if (correlation_coefficient > threshold && slope > threshold_slope)
                if (correlation_coefficient > threshold)
                {
                    std::cout << " removed" << std::endl;
                    res_batch_1.erase(res_batch_1.begin() + i);
                }
                else {
                    std::cout << std::endl;
                }
            }
            res_batch.push_back(res_batch_1);
        // }
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }*/
    // removeTheSticker(res_batch_1, depthImg, 0.95, cv::Mat matrix);

    // *******************************************************************************

    // Draw bounding boxes
    // std::cout << " after size: " << res_batch.size() << std::endl;
    draw_bbox(img, res_batch);
}

void yolov8_stop() {
    if (status == 0) {
        std::cerr << "yolov8 stop fail: yolov8 is not running..." << std::endl;
        return;
    }
    status = 0;
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(device_buffers[0]));
    CUDA_CHECK(cudaFree(device_buffers[1]));
    CUDA_CHECK(cudaFree(decode_ptr_device));
    delete[] decode_ptr_host;
    delete[] output_buffer_host;
    cuda_preprocess_destroy();
    // Destroy the engine
    delete context;
    delete engine;
    delete runtime;
}
