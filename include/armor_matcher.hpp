#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_types.hpp"
#include "light_bar_filter.hpp"

namespace auto_aim
{

struct ArmorMatcherParams
{
    double max_light_length_ratio = 3.0;
    double max_light_angle_diff_deg = 20.0;
    double max_light_center_y_diff = 400.0;
    double min_center_distance_ratio = 1.0;
    double max_center_distance_ratio = 5.0;
    double large_armor_min_center_distance_ratio = 3.2;
};

struct ArmorCandidate
{
    Armor armor;
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

private:
    ArmorMatcherParams params_;
};

} // namespace auto_aim
