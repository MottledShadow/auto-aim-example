#pragma once

#include <string>
#include <vector>

#include <opencv2/dnn.hpp>

#include "detector_types.hpp"

namespace auto_aim::detector
{

struct NumberClassifierParams
{
    std::string modelPath = "model/mlp.onnx";
    std::string labelPath = "model/label.txt";
    int warpHeight = 28;
    int lightLength = 12;
    int smallArmorWidth = 32;
    int largeArmorWidth = 54;
    int roiWidth = 20;
    double confidenceThreshold = 0.7;
};

class NumberClassifier
{
public:
    explicit NumberClassifier(const NumberClassifierParams& params = {});

    const std::string& error() const { return error_; }

    std::vector<Armor> classify(const cv::Mat& frame, const std::vector<Armor>& armors);

    struct Diagnosis
    {
        cv::Mat roi;
        cv::Mat binary;
        std::vector<float> probs;
        int classIndex = 0;
        float confidence = 0.0F;
    };
    Diagnosis diagnose(const cv::Mat& frame, const Armor& armor);

    const std::vector<std::string>& labels() const { return labels_; }

private:
    NumberClassifierParams params_;
    cv::dnn::Net net_;
    std::vector<std::string> labels_;
    std::string error_;
};

}
