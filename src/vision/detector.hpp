#pragma once
#include <opencv2/dnn.hpp>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <optional>

namespace vision {

struct Detection {
    cv::Rect  box;
    int       class_id;
    float     confidence;
    // Centre normalised to [-1, 1] — fed directly into EKF as measurement
    float     cx_norm;
    float     cy_norm;
};

class YoloDetector {
public:
    YoloDetector(const std::string& cfg,
                 const std::string& weights,
                 const std::string& names,
                 float conf_thresh  = 0.4f,
                 float nms_thresh   = 0.3f)
        : conf_thresh_{conf_thresh}
        , nms_thresh_ {nms_thresh}
    {
        net_ = cv::dnn::readNetFromDarknet(cfg, weights);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // Load class names
        std::ifstream f{names};
        std::string line;
        while (std::getline(f, line))
            if (!line.empty()) classes_.push_back(line);

        // Cache output layer names
        out_names_ = net_.getUnconnectedOutLayersNames();
    }

    // Run inference on frame, return all detections
    std::vector<Detection> detect(const cv::Mat& frame) {
        // YOLO expects 416x416 blob
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob,
            1.0/255.0, {416,416}, {0,0,0}, true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outs;
        net_.forward(outs, out_names_);

        return postprocess(frame.cols, frame.rows, outs);
    }

private:
    std::vector<Detection> postprocess(int W, int H,
                                       const std::vector<cv::Mat>& outs) {
        std::vector<int>       class_ids;
        std::vector<float>     confidences;
        std::vector<cv::Rect>  boxes;

        for (const auto& out : outs) {
            for (int i = 0; i < out.rows; ++i) {
                const float* data = reinterpret_cast<const float*>(
                    out.ptr(i));
                const cv::Mat scores{1, static_cast<int>(classes_.size()),
                                     CV_32F,
                                     const_cast<float*>(data + 5)};
                cv::Point class_pt;
                double    conf{};
                cv::minMaxLoc(scores, nullptr, &conf, nullptr, &class_pt);

                if (conf < conf_thresh_) continue;

                const int cx = static_cast<int>(data[0] * W);
                const int cy = static_cast<int>(data[1] * H);
                const int w  = static_cast<int>(data[2] * W);
                const int h  = static_cast<int>(data[3] * H);

                boxes.push_back({cx - w/2, cy - h/2, w, h});
                confidences.push_back(static_cast<float>(conf));
                class_ids.push_back(class_pt.x);
            }
        }

        // Non-maximum suppression
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences,
                          conf_thresh_, nms_thresh_, indices);

        std::vector<Detection> result;
        for (int idx : indices) {
            Detection d;
            d.box        = boxes[idx];
            d.class_id   = class_ids[idx];
            d.confidence = confidences[idx];
            d.cx_norm    = (static_cast<float>(d.box.x + d.box.width/2)
                           / static_cast<float>(W)) * 2.f - 1.f;
            d.cy_norm    = (static_cast<float>(d.box.y + d.box.height/2)
                           / static_cast<float>(H)) * 2.f - 1.f;
            result.push_back(d);
        }
        return result;
    }

    cv::dnn::Net             net_;
    std::vector<std::string> classes_;
    std::vector<std::string> out_names_;
    float                    conf_thresh_;
    float                    nms_thresh_;
};

} // namespace vision
