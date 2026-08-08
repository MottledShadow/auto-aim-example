#include "vision_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

constexpr double kEpsilon = 1e-6;

bool sameKnownColor(const LightBar& a, const LightBar& b)
{
    return a.color != LightColor::Unknown && a.color == b.color;
}

bool containsPoint(const std::vector<cv::Point2f>& region, const cv::Point2f& point)
{
    return cv::pointPolygonTest(region, point, false) >= 0.0;
}

bool hasLightBetween(
    const std::vector<LightBar>& lights,
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

        const LightBar& light = lights[i];
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
    return Armor{left, right, center, type};
}

} // namespace

ArmorMatchResult matchArmors(const std::vector<LightBar>& light_bars, const ArmorMatcherParams& params)
{
    ArmorMatchResult result;
    const std::vector<LightBar>& lights = light_bars;

    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lights.size(); ++j)
        {
            std::size_t left_index = i;
            std::size_t right_index = j;
            if (lights[right_index].center.x < lights[left_index].center.x)
            {
                std::swap(left_index, right_index);
            }

            const LightBar& left = lights[left_index];
            const LightBar& right = lights[right_index];

            if (!sameKnownColor(left, right))
            {
                continue;
            }

            const double min_length = std::min(left.length, right.length);
            if (min_length <= kEpsilon)
            {
                continue;
            }

            const double length_ratio = std::max(left.length, right.length) / min_length;
            const double angle_diff = std::abs(left.angle - right.angle);
            const double center_y_diff = std::abs(left.center.y - right.center.y);
            const double average_height = (left.length + right.length) * 0.5;
            if (average_height <= kEpsilon)
            {
                continue;
            }

            const double center_distance_ratio =
                std::hypot(left.center.x - right.center.x, left.center.y - right.center.y) /
                average_height;

            if (length_ratio > params.max_light_length_ratio ||
                angle_diff > params.max_light_angle_diff_deg ||
                center_y_diff > params.max_light_center_y_diff ||
                center_distance_ratio < params.min_center_distance_ratio ||
                center_distance_ratio > params.max_center_distance_ratio)
            {
                continue;
            }

            const std::vector<cv::Point2f> region = armorCorners(left, right);
            if (hasLightBetween(lights, left_index, right_index, region))
            {
                continue;
            }

            result.candidates.emplace_back(
                makeArmor(left, right, classifyArmor(center_distance_ratio, params)));
        }
    }

    return result;
}

} // namespace auto_aim
