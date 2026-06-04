#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

struct ArmorPreprocessParams
{
    int binary_threshold = 75;
    int open_kernel_size = 5;
    int close_kernel_size = 5;
    int morph_iterations = 1;
};

struct ArmorPreprocessResult
{
    cv::Mat binary;
    std::vector<std::vector<cv::Point>> candidate_contours;
    std::vector<cv::RotatedRect> candidate_rects;
    std::vector<cv::Vec4f> candidate_center_lines;
    std::vector<double> candidate_areas;
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
