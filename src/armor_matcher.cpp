#include "armor_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

constexpr double kMinLength = 1e-6;

void validateParams(const ArmorMatcherParams& params)
{
    if (params.max_light_length_ratio < 1.0)
    {
        throw std::invalid_argument("max_light_length_ratio must be >= 1");
    }
    if (params.max_light_angle_diff_deg < 0.0)
    {
        throw std::invalid_argument("max_light_angle_diff_deg must be non-negative");
    }
    if (params.max_light_center_y_diff < 0.0)
    {
        throw std::invalid_argument("max_light_center_y_diff must be non-negative");
    }
    if (params.min_center_distance_ratio < 0.0 ||
        params.max_center_distance_ratio < params.min_center_distance_ratio)
    {
        throw std::invalid_argument("invalid center distance ratio range");
    }
    if (params.large_armor_min_center_distance_ratio < 0.0)
    {
        throw std::invalid_argument("large armor threshold must be non-negative");
    }
}

double distance(const cv::Point2f& a, const cv::Point2f& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool sameKnownColor(const LightBar& a, const LightBar& b)
{
    return a.color != LightColor::Unknown && a.color == b.color;
}

std::vector<cv::Point2f> pairRegion(const LightBar& left, const LightBar& right)
{
    return {
        left.top,
        right.top,
        right.bottom,
        left.bottom,
    };
}

bool containsPoint(const std::vector<cv::Point2f>& region, const cv::Point2f& point)
{
    return cv::pointPolygonTest(region, point, false) >= 0.0;
}

bool hasLightBetween(
    const std::vector<LightBarCandidate>& lights,
    std::size_t left_index,
    std::size_t right_index,
    const std::vector<cv::Point2f>& region)
{
    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        if (i == left_index || i == right_index)
        {
            continue;
        }

        const LightBar& light = lights[i].light_bar;
        if (containsPoint(region, light.top) ||
            containsPoint(region, light.bottom) ||
            containsPoint(region, light.center))
        {
            return true;
        }
    }
    return false;
}

ArmorType classifyArmor(double center_distance_ratio, const ArmorMatcherParams& params)
{
    if (center_distance_ratio >= params.large_armor_min_center_distance_ratio)
    {
        return ArmorType::Large;
    }
    return ArmorType::Small;
}

Armor makeArmor(const LightBar& left, const LightBar& right, ArmorType type)
{
    const cv::Point2f center = (left.center + right.center) * 0.5F;
    return Armor(left, right, center, type);
}

} // namespace

ArmorMatcher::ArmorMatcher(ArmorMatcherParams params) : params_(params)
{
    validateParams(params_);
}

ArmorMatchResult ArmorMatcher::match(const LightBarFilterResult& light_bars) const
{
    ArmorMatchResult result;
    const std::vector<LightBarCandidate>& lights = light_bars.candidates;

    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lights.size(); ++j)
        {
            std::size_t left_index = i;
            std::size_t right_index = j;
            if (lights[right_index].light_bar.center.x < lights[left_index].light_bar.center.x)
            {
                std::swap(left_index, right_index);
            }

            const LightBar& left = lights[left_index].light_bar;
            const LightBar& right = lights[right_index].light_bar;

            if (!sameKnownColor(left, right))
            {
                continue;
            }

            const double min_length = std::min(left.length, right.length);
            if (min_length <= kMinLength)
            {
                continue;
            }

            const double length_ratio = std::max(left.length, right.length) / min_length;
            const double angle_diff = std::abs(left.angle - right.angle);
            const double center_y_diff = std::abs(left.center.y - right.center.y);
            const double average_height = (left.length + right.length) * 0.5;
            if (average_height <= kMinLength)
            {
                continue;
            }

            const double center_distance_ratio =
                distance(left.center, right.center) / average_height;

            if (length_ratio > params_.max_light_length_ratio)
            {
                continue;
            }
            if (angle_diff > params_.max_light_angle_diff_deg)
            {
                continue;
            }
            if (center_y_diff > params_.max_light_center_y_diff)
            {
                continue;
            }
            if (center_distance_ratio < params_.min_center_distance_ratio ||
                center_distance_ratio > params_.max_center_distance_ratio)
            {
                continue;
            }

            const std::vector<cv::Point2f> region = pairRegion(left, right);
            if (hasLightBetween(lights, left_index, right_index, region))
            {
                continue;
            }

            ArmorCandidate candidate;
            candidate.armor = makeArmor(left, right, classifyArmor(center_distance_ratio, params_));
            candidate.light_length_ratio = length_ratio;
            candidate.light_angle_diff_deg = angle_diff;
            candidate.light_center_y_diff = center_y_diff;
            candidate.center_distance_ratio = center_distance_ratio;
            result.candidates.emplace_back(candidate);
        }
    }

    return result;
}

const ArmorMatcherParams& ArmorMatcher::params() const
{
    return params_;
}

} // namespace auto_aim
