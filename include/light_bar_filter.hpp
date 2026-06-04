#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_preprocessor.hpp"
#include "armor_types.hpp"

namespace auto_aim
{

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

struct LightBarCandidate
{
    LightBar light_bar;
    double area = 0.0;
};

struct LightBarFilterResult
{
    std::vector<LightBarCandidate> candidates;
};

class LightBarFilter
{
public:
    explicit LightBarFilter(LightBarFilterParams params = {});

    LightBarFilterResult filter(
        const cv::Mat& frame,
        const ArmorPreprocessResult& preprocess) const;

private:
    LightBarFilterParams params_;
};

} // namespace auto_aim
