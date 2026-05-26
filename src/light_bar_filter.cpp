#include "light_bar_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinSide = 1e-6;

void validateParams(const LightBarFilterParams& params)
{
    if (params.min_area < 0.0 || params.max_area < params.min_area)
    {
        throw std::invalid_argument("invalid light bar area range");
    }
    if (params.min_aspect_ratio < 0.0 || params.max_aspect_ratio < params.min_aspect_ratio)
    {
        throw std::invalid_argument("invalid light bar aspect ratio range");
    }
    if (params.min_line_angle_deg < 0.0 || params.max_line_angle_deg > 90.0 ||
        params.max_line_angle_deg < params.min_line_angle_deg)
    {
        throw std::invalid_argument("invalid light bar line angle range");
    }
    if (params.min_fill_ratio < 0.0 || params.max_fill_ratio < params.min_fill_ratio)
    {
        throw std::invalid_argument("invalid light bar fill ratio range");
    }
}

double edgeLength(const cv::Point2f& a, const cv::Point2f& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

cv::Point2f midpoint(const cv::Point2f& a, const cv::Point2f& b)
{
    return (a + b) * 0.5F;
}

double lineAngleFromVerticalDeg(const cv::Vec4f& line)
{
    const double vx = static_cast<double>(line[0]);
    const double vy = static_cast<double>(line[1]);
    const double norm = std::hypot(vx, vy);
    if (norm <= kMinSide)
    {
        return 90.0;
    }

    const double cos_to_vertical = std::clamp(std::abs(vy) / norm, 0.0, 1.0);
    return std::acos(cos_to_vertical) * 180.0 / kPi;
}

bool inRange(double value, double min_value, double max_value)
{
    return value >= min_value && value <= max_value;
}

LightColor detectLightColor(const cv::Mat& frame, const std::vector<cv::Point>& contour)
{
    if (frame.empty() || frame.channels() < 3)
    {
        return LightColor::Unknown;
    }

    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours{contour};
    cv::drawContours(mask, contours, 0, cv::Scalar(255), cv::FILLED);

    const double pixel_count = static_cast<double>(cv::countNonZero(mask));
    if (pixel_count <= 0.0)
    {
        return LightColor::Unknown;
    }

    const cv::Scalar mean = cv::mean(frame, mask);
    const double blue_sum = mean[0] * pixel_count;
    const double red_sum = mean[2] * pixel_count;
    if (red_sum > blue_sum)
    {
        return LightColor::Red;
    }
    if (blue_sum > red_sum)
    {
        return LightColor::Blue;
    }
    return LightColor::Unknown;
}

LightBar makeLightBar(
    const cv::RotatedRect& rect,
    double line_angle_deg,
    LightColor color)
{
    const float length = std::max(rect.size.width, rect.size.height);
    const float width = std::min(rect.size.width, rect.size.height);
    const cv::Point2f center = rect.center;

    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f p1;
    cv::Point2f p2;

    if (edgeLength(vertices[0], vertices[1]) <= edgeLength(vertices[1], vertices[2]))
    {
        p1 = midpoint(vertices[0], vertices[1]);
        p2 = midpoint(vertices[2], vertices[3]);
    }
    else
    {
        p1 = midpoint(vertices[1], vertices[2]);
        p2 = midpoint(vertices[3], vertices[0]);
    }

    if (p1.y > p2.y)
    {
        std::swap(p1, p2);
    }

    return LightBar(
        color,
        p1,
        p2,
        center,
        length,
        width,
        static_cast<float>(line_angle_deg));
}

} // namespace

LightBarFilter::LightBarFilter(LightBarFilterParams params) : params_(params)
{
    validateParams(params_);
}

LightBarFilterResult LightBarFilter::filter(
    const cv::Mat& frame,
    const ArmorPreprocessResult& preprocess) const
{
    LightBarFilterResult result;
    const std::size_t count = std::min(
        preprocess.candidate_contours.size(),
        std::min(
            preprocess.candidate_areas.size(),
            std::min(preprocess.candidate_rects.size(), preprocess.candidate_center_lines.size())));
    result.candidates.reserve(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        const std::vector<cv::Point>& contour = preprocess.candidate_contours[i];
        const cv::RotatedRect& rect = preprocess.candidate_rects[i];
        const cv::Vec4f& line = preprocess.candidate_center_lines[i];
        const double area = preprocess.candidate_areas[i];
        const double long_side = std::max(rect.size.width, rect.size.height);
        const double short_side = std::min(rect.size.width, rect.size.height);
        if (short_side <= kMinSide)
        {
            continue;
        }

        const double rect_area = long_side * short_side;
        if (rect_area <= kMinSide)
        {
            continue;
        }

        const double aspect_ratio = long_side / short_side;
        const double line_angle_deg = lineAngleFromVerticalDeg(line);
        const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

        if (!inRange(area, params_.min_area, params_.max_area))
        {
            continue;
        }
        if (!inRange(aspect_ratio, params_.min_aspect_ratio, params_.max_aspect_ratio))
        {
            continue;
        }
        if (!inRange(line_angle_deg, params_.min_line_angle_deg, params_.max_line_angle_deg))
        {
            continue;
        }
        if (!inRange(fill_ratio, params_.min_fill_ratio, params_.max_fill_ratio))
        {
            continue;
        }

        LightBarCandidate candidate;
        const LightColor color = detectLightColor(frame, contour);
        candidate.light_bar = makeLightBar(rect, line_angle_deg, color);
        candidate.area = area;
        candidate.rect_area = rect_area;
        candidate.aspect_ratio = aspect_ratio;
        candidate.line_angle_deg = line_angle_deg;
        candidate.fill_ratio = fill_ratio;
        result.candidates.emplace_back(candidate);
    }

    return result;
}

const LightBarFilterParams& LightBarFilter::params() const
{
    return params_;
}

} // namespace auto_aim
