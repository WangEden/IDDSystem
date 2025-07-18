#include "postprocess.h"
#include "utils.h"

Queue CC004;
int CC004_valid;
int CC004_MAX;
Queue Item[21];
int valid[21];
int Max[21];

int num_cnt[21][50];
int current_num[21];

cv::Rect get_rect(cv::Mat& img, float bbox[4]) {
    float l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    // std::cout << "cols rows: " << img.cols << " " << img.rows << std::endl;

    if (r_h > r_w) {
        l = bbox[0];
        r = bbox[2];
        t = bbox[1] - (kInputH - r_w * img.rows) / 2;
        b = bbox[3] - (kInputH - r_w * img.rows) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - (kInputW - r_h * img.cols) / 2;
        r = bbox[2] - (kInputW - r_h * img.cols) / 2;
        t = bbox[1];
        b = bbox[3];
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    return cv::Rect(round(l), round(t), round(r - l), round(b - t));
}

cv::Rect get_rect_adapt_landmark(cv::Mat& img, float bbox[4], float lmk[51]) {
    int l, r, t, b;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    if (r_h > r_w) {
        l = bbox[0] / r_w;
        r = bbox[2] / r_w;
        t = (bbox[1] - (kInputH - r_w * img.rows) / 2) / r_w;
        b = (bbox[3] - (kInputH - r_w * img.rows) / 2) / r_w;
        for (int i = 0; i < 51; i += 3) {
            lmk[i] /= r_w;
            lmk[i + 1] = (lmk[i + 1] - (kInputH - r_w * img.rows) / 2) / r_w;
            // lmk[i + 2]
        }
    } else {
        l = (bbox[0] - (kInputW - r_h * img.cols) / 2) / r_h;
        r = (bbox[2] - (kInputW - r_h * img.cols) / 2) / r_h;
        t = bbox[1] / r_h;
        b = bbox[3] / r_h;
        for (int i = 0; i < 51; i += 3) {
            lmk[i] = (lmk[i] - (kInputW - r_h * img.cols) / 2) / r_h;
            lmk[i + 1] /= r_h;
            // lmk[i + 2]
        }
    }
    return cv::Rect(l, t, r - l, b - t);
}

static float iou(float lbox[4], float rbox[4]) {
    float interBox[] = {
            (std::max)(lbox[0], rbox[0]),
            (std::min)(lbox[2], rbox[2]),
            (std::max)(lbox[1], rbox[1]),
            (std::min)(lbox[3], rbox[3]),
    };

    if (interBox[2] > interBox[3] || interBox[0] > interBox[1])
        return 0.0f;

    float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
    float unionBoxS = (lbox[2] - lbox[0]) * (lbox[3] - lbox[1]) + (rbox[2] - rbox[0]) * (rbox[3] - rbox[1]) - interBoxS;
    return interBoxS / unionBoxS;
}

static bool cmp(const Detection& a, const Detection& b) {
    if (a.conf == b.conf) {
        return a.bbox[0] < b.bbox[0];
    }
    return a.conf > b.conf;
}

void nms(std::vector<Detection>& res, float* output, float conf_thresh, float nms_thresh) {
    int det_size = sizeof(Detection) / sizeof(float);
    std::map<float, std::vector<Detection>> m;

    for (int i = 0; i < output[0]; i++) {
        if (output[1 + det_size * i + 4] <= conf_thresh)
            continue;
        Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (m.count(det.class_id) == 0)
            m.emplace(det.class_id, std::vector<Detection>());
        m[det.class_id].push_back(det);
    }
    for (auto it = m.begin(); it != m.end(); it++) {
        auto& dets = it->second;
        std::sort(dets.begin(), dets.end(), cmp);
        for (size_t m = 0; m < dets.size(); ++m) {
            auto& item = dets[m];
            res.push_back(item);
            for (size_t n = m + 1; n < dets.size(); ++n) {
                if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
                    dets.erase(dets.begin() + n);
                    --n;
                }
            }
        }
    }
}

void batch_nms(std::vector<std::vector<Detection>>& res_batch, float* output, int batch_size, int output_size,
               float conf_thresh, float nms_thresh) {
    res_batch.resize(batch_size);
    for (int i = 0; i < batch_size; i++) {
        nms(res_batch[i], &output[i * output_size], conf_thresh, nms_thresh);
    }
}

void process_decode_ptr_host(std::vector<Detection>& res, const float* decode_ptr_host, int bbox_element, cv::Mat& img,
                             int count) {
    Detection det;
    for (int i = 0; i < count; i++) {
        int basic_pos = 1 + i * bbox_element;
        int keep_flag = decode_ptr_host[basic_pos + 6];
        if (keep_flag == 1) {
            det.bbox[0] = decode_ptr_host[basic_pos + 0];
            det.bbox[1] = decode_ptr_host[basic_pos + 1];
            det.bbox[2] = decode_ptr_host[basic_pos + 2];
            det.bbox[3] = decode_ptr_host[basic_pos + 3];
            det.conf = decode_ptr_host[basic_pos + 4];
            det.class_id = decode_ptr_host[basic_pos + 5];
            res.push_back(det);
        }
    }
}

void batch_process(std::vector<std::vector<Detection>>& res_batch, const float* decode_ptr_host, int batch_size,
                   int bbox_element, const std::vector<cv::Mat>& img_batch) {
    res_batch.resize(batch_size);
    int count = static_cast<int>(*decode_ptr_host);
    count = std::min(count, kMaxNumOutputBbox);
    for (int i = 0; i < batch_size; i++) {
        auto& img = const_cast<cv::Mat&>(img_batch[i]);
        process_decode_ptr_host(res_batch[i], &decode_ptr_host[i * count], bbox_element, img, count);
    }
}

void draw_bbox(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch) {
    for (int i = 0; i < 21; i++) {
        current_num[i] = 0;
    }
    for (size_t i = 0; i < img_batch.size(); i++) 
    {
        auto& res = res_batch[i];
        cv::Mat img = img_batch[i];
        for (size_t j = 0; j < res.size(); j++) 
        {
            cv::Rect r = get_rect(img, res[j].bbox);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            //convert the id to class_name
            std::string nametext;
            switch((int)res[j].class_id)
            {
                case 0:
                nametext = "Insulator";current_num[0]++;break;
                case 1:
                nametext = "ShellBroken";current_num[1]++;break;
                case 2:
                nametext = "FlashOver";current_num[2]++;break;
                case 3:
                nametext = "NoIssue";current_num[3]++;break;
                case 4:
                nametext = "CB001";current_num[4]++;break;
                case 5:
                nametext = "CB002";current_num[5]++;break;
                case 6:
                nametext = "CB003";current_num[6]++;break;
                case 7:
                nametext = "CB004";current_num[7]++;break;
                case 8:
                nametext = "CC001";current_num[8]++;break;
                case 9:
                nametext = "CC002";current_num[9]++;break;
                case 10:
                nametext = "CC003";current_num[10]++;break;
                case 11:
                nametext = "CC004";current_num[11]++;break;
                case 12:
                nametext = "CD001";current_num[12]++;break;
                case 13:
                nametext = "CD002";current_num[13]++;break;
                case 14:
                nametext = "CD003";current_num[14]++;break;
                case 15:
                nametext = "CD004";current_num[15]++;break;
                case 16:
                nametext = "TABLE";current_num[16]++;break;
                case 17:
                nametext = "W001";current_num[17]++;break;
                case 18:
                nametext = "W002";current_num[18]++;break;
                case 19:
                nametext = "W003";current_num[19]++;break;
                case 20:
                nametext = "W004";current_num[20]++;break;
                default:
                break;
            }
            cv::putText(img, nametext, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            // cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
            //             cv::Scalar(0xFF, 0xFF, 0xFF), 2);
        }
    }
    for (int i = 0; i < 21; i++) 
    {
        Item[i].enqueue(current_num[i]);
        valid[i] = Item[i].checkUniformity();
        if((valid[i] > 0) && (valid[i] > Max[i]))
        {
            Max[i] =  valid[i];
        }
    }
    //CC004.enqueue(current_num[11]);
    // CC004_valid = CC004.checkUniformity();
    // if((CC004_valid > 0) && (CC004_valid > CC004_MAX))
    // {
    //     CC004_MAX =  CC004_valid;
    // }
    
    // for (int i = 0; i < 21; i++) {
    //     num_cnt[i][current_num[i]]++;
    // }
}

void draw_bbox_keypoints_line(std::vector<cv::Mat>& img_batch, std::vector<std::vector<Detection>>& res_batch) {
    const std::vector<std::pair<int, int>> skeleton_pairs = {
            {0, 1}, {0, 2},  {0, 5}, {0, 6},  {1, 2},   {1, 3},   {2, 4},   {5, 6},   {5, 7},  {5, 11},
            {6, 8}, {6, 12}, {7, 9}, {8, 10}, {11, 12}, {11, 13}, {12, 14}, {13, 15}, {14, 16}};

    for (size_t i = 0; i < img_batch.size(); i++) {
        auto& res = res_batch[i];
        cv::Mat img = img_batch[i];
        for (size_t j = 0; j < res.size(); j++) {
            cv::Rect r = get_rect_adapt_landmark(img, res[j].bbox, res[j].keypoints);
            cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
            cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2,
                        cv::Scalar(0xFF, 0xFF, 0xFF), 2);

            for (int k = 0; k < 51; k += 3) {
                if (res[j].keypoints[k + 2] > 0.5) {
                    cv::circle(img, cv::Point((int)res[j].keypoints[k], (int)res[j].keypoints[k + 1]), 3,
                               cv::Scalar(0, 0x27, 0xC1), -1);
                }
            }

            for (const auto& bone : skeleton_pairs) {
                int kp1_idx = bone.first * 3;
                int kp2_idx = bone.second * 3;
                if (res[j].keypoints[kp1_idx + 2] > 0.5 && res[j].keypoints[kp2_idx + 2] > 0.5) {
                    cv::Point p1((int)res[j].keypoints[kp1_idx], (int)res[j].keypoints[kp1_idx + 1]);
                    cv::Point p2((int)res[j].keypoints[kp2_idx], (int)res[j].keypoints[kp2_idx + 1]);
                    cv::line(img, p1, p2, cv::Scalar(0, 0x27, 0xC1), 2);
                }
            }
        }
    }
}

cv::Mat scale_mask(cv::Mat mask, cv::Mat img) {
    int x, y, w, h;
    float r_w = kInputW / (img.cols * 1.0);
    float r_h = kInputH / (img.rows * 1.0);
    if (r_h > r_w) {
        w = kInputW;
        h = r_w * img.rows;
        x = 0;
        y = (kInputH - h) / 2;
    } else {
        w = r_h * img.cols;
        h = kInputH;
        x = (kInputW - w) / 2;
        y = 0;
    }
    cv::Rect r(x, y, w, h);
    cv::Mat res;
    cv::resize(mask(r), res, img.size());
    return res;
}

void draw_mask_bbox(cv::Mat& img, std::vector<Detection>& dets, std::vector<cv::Mat>& masks,
                    std::unordered_map<int, std::string>& labels_map) {
    static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A, 0x92CC17,
                                           0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF, 0x344593, 0x6473FF,
                                           0x0018EC, 0x8438FF, 0x520085, 0xCB38FF, 0xFF95C8, 0xFF37C7};
    for (size_t i = 0; i < dets.size(); i++) {
        cv::Mat img_mask = scale_mask(masks[i], img);
        auto color = colors[(int)dets[i].class_id % colors.size()];
        auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

        cv::Rect r = get_rect(img, dets[i].bbox);
        for (int x = r.x; x < r.x + r.width; x++) {
            for (int y = r.y; y < r.y + r.height; y++) {
                float val = img_mask.at<float>(y, x);
                if (val <= 0.5)
                    continue;
                img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
                img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
                img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
            }
        }

        cv::rectangle(img, r, bgr, 2);

        // Get the size of the text
        cv::Size textSize =
                cv::getTextSize(labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf),
                                cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL);
        // Set the top left corner of the rectangle
        cv::Point topLeft(r.x, r.y - textSize.height);

        // Set the bottom right corner of the rectangle
        cv::Point bottomRight(r.x + textSize.width, r.y + textSize.height);

        // Set the thickness of the rectangle lines
        int lineThickness = 2;

        // Draw the rectangle on the image
        cv::rectangle(img, topLeft, bottomRight, bgr, -1);

        cv::putText(img, labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf),
                    cv::Point(r.x, r.y + 4), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar::all(0xFF), 2);
    }
}
