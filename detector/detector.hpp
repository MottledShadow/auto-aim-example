#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

// ========== 调参结构体 ==========

struct PreprocessParams
{
    int binary_threshold = 90;  // 灰度二值化阈值
};

struct LightBarFilterParams
{
    double min_area = 10.0;
    double max_area = 6000.0;
    double min_aspect_ratio = 2.0;
    double max_aspect_ratio = 15.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 15.0;
    double min_fill_ratio = 0.5;
    double max_fill_ratio = 1.0;
    LightColor target_color = LightColor::Red;  // 目标灯条颜色，非此色的灯条舍弃
};

struct LightBarMatcherParams
{
    double max_light_length_ratio = 1.5;                 // 两灯条长度比上限（长/短）
    double max_light_angle_diff_deg = 10.0;              // 两灯条角度差上限（度）// TODO 上车重点调整
    double max_light_center_y_diff = 30.0;              // 两灯条中心 y 差上限（像素）
    double min_center_distance_ratio = 2.0;              // 中心距/平均灯条长 下限
    double max_center_distance_ratio = 6.0;              // 中心距/平均灯条长 上限
    double large_armor_min_center_distance_ratio = 4.0;  // 中心距比 ≥ 此值判大装甲
};

// ========== 数据结构体 ==========

struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f center_line;
    double area = 0.0;
};

struct PreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

struct LightBar
{
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float angle = 0.0F;
};

enum class ArmorType
{
    Unknown = 0,
    Small,
    Large,
};

struct Armor
{
    LightBar left_light;
    LightBar right_light;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;
    std::string number;          // 分类得到的数字/标签，空表示未分类
    float confidence = 0.0F;     // 分类 softmax 最大概率
};

// ========== 检测器 ==========

// 无状态的检测器：把三个参数结构体收成公开成员，三个阶段各一方法
class Detector
{
public:
    PreprocessParams preprocess_params;
    LightBarFilterParams filter_params;
    LightBarMatcherParams matcher_params;

    PreprocessResult preprocess(const cv::Mat& frame) const;

    std::vector<LightBar> filterLightBars(
        const cv::Mat& frame,
        const PreprocessResult& pre) const;

    std::vector<Armor> matchArmors(const std::vector<LightBar>& light_bars) const;
};

} // namespace auto_aim

// 伞头：下游只需 include "detector.hpp" 即可拿到分类与 PnP 接口
// 放在文件末尾，确保上面的共享类型（Armor 等）先于子头可见，化解环形包含
#include "classifier.hpp"
#include "pnp.hpp"
