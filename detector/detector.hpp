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

} // namespace auto_aim
