#pragma once

#include <string>
#include <vector>

#include <opencv2/dnn.hpp>

#include "detector.hpp"

namespace auto_aim
{

// 数字分类参数：矫正尺寸、ROI、模型/标签路径、置信度阈值
struct NumberClassifierParams
{
    std::string model_path = "model/mlp.onnx";
    std::string label_path = "model/label.txt";
    int warp_height = 28;         // 矫正后高度
    int light_length = 12;        // 灯条长度占 28 像素中间的 12 像素
    int small_armor_width = 32;   // 小装甲矫正宽度
    int large_armor_width = 54;   // 大装甲矫正宽度
    int roi_width = 20;           // 取中间 20×28 ROI（高沿用 warp_height）
    double confidence_threshold = 0.7;  // 最大概率低于此值丢弃
};

// 数字分类器：ONNX 网络 + 标签表；error 为空表示加载成功
struct NumberClassifier
{
    cv::dnn::Net net;
    std::vector<std::string> labels;
    std::string error;
};

NumberClassifier loadClassifier(const NumberClassifierParams& params = {});

std::vector<Armor> classifyArmors(
    const cv::Mat& frame,
    const std::vector<Armor>& armors,
    NumberClassifier& classifier,
    const NumberClassifierParams& params = {});

} // namespace auto_aim
