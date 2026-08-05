#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

struct PreprocessParams
{
    int binary_threshold = 20;
    int open_kernel_size = 5;
    int close_kernel_size = 5;
    int morph_iterations = 1;
};

struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f center_line;
    double area = 0.0;
};

struct ArmorPreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

ArmorPreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params = {});

} // namespace auto_aim
