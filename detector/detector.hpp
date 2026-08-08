#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

struct PreprocessParams
{
    int binary_threshold = 125;
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
};

PreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params = {});

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

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
    double min_area = 30.0;
    double max_area = 20000.0;
    double min_aspect_ratio = 2.0;
    double max_aspect_ratio = 30.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 30.0;
    double min_fill_ratio = 0.75;
    double max_fill_ratio = 1.0;
};

std::vector<LightBar> filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& preprocess,
    const LightBarFilterParams& params = {});

} // namespace auto_aim
