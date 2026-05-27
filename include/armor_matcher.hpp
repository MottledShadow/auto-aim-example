#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_types.hpp"
#include "light_bar_filter.hpp"

namespace auto_aim
{

struct ArmorMatcherParams
{
    double max_light_length_ratio = 2.0;
    double max_light_angle_diff_deg = 10.0;
    double max_light_center_y_diff = 40.0;
    double min_center_distance_ratio = 0.5;
    double max_center_distance_ratio = 8.0;
    double large_armor_min_center_distance_ratio = 3.2;
};

struct ArmorCandidate
{
    Armor armor;
    double light_length_ratio = 0.0;
    double light_angle_diff_deg = 0.0;
    double light_center_y_diff = 0.0;
    double center_distance_ratio = 0.0;
};

struct ArmorMatchResult
{
    std::vector<ArmorCandidate> candidates;
};

class ArmorMatcher
{
public:
    explicit ArmorMatcher(ArmorMatcherParams params = {});

    ArmorMatchResult match(const LightBarFilterResult& light_bars) const;

    const ArmorMatcherParams& params() const;

private:
    ArmorMatcherParams params_;
};

} // namespace auto_aim
