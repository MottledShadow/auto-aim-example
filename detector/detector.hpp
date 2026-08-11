#pragma once

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

enum class BinaryMethod
{
    Gray,            // 灰度阈值二值化
    ChannelSubtract, // 红蓝通道相减二值化
};

struct PreprocessParams
{
    BinaryMethod method = BinaryMethod::ChannelSubtract; // 默认走通道相减
    LightColor target_color = LightColor::Blue;           // 通道相减时算哪一路（Red→R-B，Blue→B-R）
    int binary_threshold = 125;              // 灰度法阈值
    int channel_sub_threshold_red = 55;      // 通道相减法·红（R-B）阈值
    int channel_sub_threshold_blue = 75;     // 通道相减法·蓝（B-R）阈值，蓝灯条要设更高
};

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
    BinaryMethod method = BinaryMethod::ChannelSubtract;  // 该二值图用的方法
    LightColor target_color = LightColor::Red;            // 通道相减法时的目标色
};

PreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params = {});

struct LightBar
{
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float width = 0.0F;
    float angle = 0.0F;
    float area = 0.0F;
};

struct LightBarFilterParams
{
    double min_area = 20.0;
    double max_area = 6000.0;
    double min_aspect_ratio = 2.0;
    double max_aspect_ratio = 10.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 10.0;
    double min_fill_ratio = 0.4;
    double max_fill_ratio = 1.0;
};

std::vector<LightBar> filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& preprocess,
    const LightBarFilterParams& params = {});

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
};

struct LightBarMatcherParams
{
    double max_light_length_ratio = 3.0;                 // 两灯条长度比上限（长/短）
    double max_light_angle_diff_deg = 20.0;              // 两灯条角度差上限（度）
    double max_light_center_y_diff = 400.0;              // 两灯条中心 y 差上限（像素）
    double min_center_distance_ratio = 1.0;              // 中心距/平均灯条长 下限
    double max_center_distance_ratio = 5.0;              // 中心距/平均灯条长 上限
    double large_armor_min_center_distance_ratio = 3.2;  // 中心距比 ≥ 此值判大装甲
};

std::vector<Armor> matchArmors(
    const std::vector<LightBar>& light_bars,
    const LightBarMatcherParams& params = {});

} // namespace auto_aim
