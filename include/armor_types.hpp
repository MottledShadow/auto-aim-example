#pragma once

#include <opencv2/core.hpp>

namespace auto_aim
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

enum class ArmorType
{
    Unknown = 0,
    Small,
    Large,
};

class LightBar
{
public:
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float width = 0.0F;
    float angle = 0.0F;

    LightBar() = default;

    LightBar(
        LightColor color,
        const cv::Point2f& top,
        const cv::Point2f& bottom,
        const cv::Point2f& center,
        float length,
        float width,
        float angle)
        : color(color),
          top(top),
          bottom(bottom),
          center(center),
          length(length),
          width(width),
          angle(angle)
    {
    }
};

class Armor
{
public:
    LightBar left_light;
    LightBar right_light;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;

    Armor() = default;

    Armor(
        const LightBar& left_light,
        const LightBar& right_light,
        const cv::Point2f& center,
        ArmorType type)
        : left_light(left_light),
          right_light(right_light),
          center(center),
          type(type)
    {
    }
};

} // namespace auto_aim
