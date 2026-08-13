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

// 数字分类器：构造时加载 ONNX 网络 + 标签表，之后每帧复用
class NumberClassifier
{
public:
    explicit NumberClassifier(const NumberClassifierParams& params = {});

    //加载失败原因，空表示加载成功
    const std::string& error() const { return error_; }

    //对配对装甲板逐块分类，返回通过四条去留规则的装甲板（写回 number/confidence）
    std::vector<Armor> classify(const cv::Mat& frame, const std::vector<Armor>& armors);

    //单块装甲的完整推理产物：矫正 ROI、二值图、全类别 softmax、最大类下标与概率（调试用）
    struct Diagnosis
    {
        cv::Mat roi;
        cv::Mat binary;
        std::vector<float> probs;
        int class_index = 0;
        float confidence = 0.0F;
    };
    Diagnosis diagnose(const cv::Mat& frame, const Armor& armor);

    //标签表（供调试打印类别名）
    const std::vector<std::string>& labels() const { return labels_; }

private:
    NumberClassifierParams params_;
    cv::dnn::Net net_;
    std::vector<std::string> labels_;
    std::string error_;
};

} // namespace auto_aim
