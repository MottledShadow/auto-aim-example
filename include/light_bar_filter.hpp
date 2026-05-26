#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_preprocessor.hpp"
#include "armor_types.hpp"

namespace auto_aim
{

struct LightBarFilterParams
{
    double min_area = 5.0;
    double max_area = 1000000.0;
    double min_aspect_ratio = 1.2;
    double max_aspect_ratio = 50.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 45.0;
    double min_fill_ratio = 0.25;
    double max_fill_ratio = 1.0;
};

struct LightBarCandidate
{
    LightBar light_bar;
    double area = 0.0;
    double rect_area = 0.0;
    double aspect_ratio = 0.0;
    double line_angle_deg = 0.0;
    double fill_ratio = 0.0;
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
        const ArmorPreprocessResult& preprocess,
        LightColor color = LightColor::Unknown) const;

    const LightBarFilterParams& params() const;

private:
    LightBarFilterParams params_;
};

} // namespace auto_aim
