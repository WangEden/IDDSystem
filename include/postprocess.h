#pragma once

#include <opencv2/opencv.hpp>
#include "NvInfer.h"
#include "types.h"
#define MAX_SIZE 4

extern int num_cnt[21][50];
extern int current_num[21];
extern int CC004_MAX;
extern int Max[21];



class Queue {
private:
    int front, rear, size;
    int queue[MAX_SIZE];

public:
    Queue() : front(0), rear(0), size(0) {}

    bool isEmpty() {
        return size == 0;
    }

    bool isFull() {
        return size == MAX_SIZE;
    }

    bool dequeue() {
        if (isEmpty()) {
            std::cout << "Queue is empty. Cannot dequeue." << std::endl;
            return false;
        }
        front = (front + 1) % MAX_SIZE;
        --size;
        return true;
    }

    bool enqueue(int data) {
        if (isFull()) {
            // 如果队列满了，先出队一个元素，然后再入队
            dequeue();
        }
        queue[rear] = data;
        rear = (rear + 1) % MAX_SIZE;
        ++size;
        return true;
    }

    // 检查队列是否满了且所有元素一致
    int checkUniformity() {
        if (!isFull()) {
            return -1;  // 队列不满
        }

        int firstValue = queue[front];
        for (int i = 1; i < MAX_SIZE; i++) {
            int currentIndex = (front + i) % MAX_SIZE;
            if (queue[currentIndex] != firstValue) {
                return -1;  // 元素不一致
            }
        }
        return firstValue;  // 返回一致的元素值
    }
};



cv::Rect get_rect(cv::Mat& img, float bbox[4]);

void nms(std::vector<Detection>& res, float* output, float conf_thresh, float nms_thresh = 0.5);

void batch_nms(std::vector<std::vector<Detection>>& batch_res, float* output, int batch_size, int output_size,
               float conf_thresh, float nms_thresh = 0.5);

void draw_bbox(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch);

void draw_bbox_keypoints_line(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch);

void batch_process(std::vector<std::vector<Detection>>& res_batch, const float* decode_ptr_host, int batch_size,
                   int bbox_element, const std::vector<cv::Mat>& img_batch);

void process_decode_ptr_host(std::vector<Detection>& res, const float* decode_ptr_host, int bbox_element, cv::Mat& img,
                             int count);

void cuda_decode(float* predict, int num_bboxes, float confidence_threshold, float* parray, int max_objects,
                 cudaStream_t stream);

void cuda_nms(float* parray, float nms_threshold, int max_objects, cudaStream_t stream);

void draw_mask_bbox(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                    std::unordered_map<int, std::string>& labels_map);
