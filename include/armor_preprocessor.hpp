#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

struct ArmorPreprocessParams
{
    int binary_threshold = 180;
    int open_kernel_size = 3;
    int close_kernel_size = 3;
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

class ArmorPreprocessor
{
public:
    explicit ArmorPreprocessor(ArmorPreprocessParams params = {});

    ArmorPreprocessResult process(const cv::Mat& frame) const;

private:
    ArmorPreprocessParams params_;
};

} // namespace auto_aim
