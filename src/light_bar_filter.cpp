#include "light_bar_filter.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinSide = 1e-6;

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
    if (frame.channels() < 3)
    {
        return LightColor::Unknown;
    }

    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255), cv::FILLED);

    const cv::Scalar mean = cv::mean(frame, mask);
    if (mean[2] > mean[0])
    {
        return LightColor::Red;
    }
    if (mean[0] > mean[2])
    {
        return LightColor::Blue;
    }
    return LightColor::Unknown;
}

LightBar makeLightBar(
    const cv::RotatedRect& rect,
    double line_angle_deg,
    LightColor color,
    double area)
{
    const float length = std::max(rect.size.width, rect.size.height);
    const float width = std::min(rect.size.width, rect.size.height);
    const cv::Point2f center = rect.center;

    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f p1;
    cv::Point2f p2;

    const double edge01 = std::hypot(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
    const double edge12 = std::hypot(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);
    if (edge01 <= edge12)
    {
        p1 = (vertices[0] + vertices[1]) * 0.5F;
        p2 = (vertices[2] + vertices[3]) * 0.5F;
    }
    else
    {
        p1 = (vertices[1] + vertices[2]) * 0.5F;
        p2 = (vertices[3] + vertices[0]) * 0.5F;
    }

    if (p1.y > p2.y)
    {
        std::swap(p1, p2);
    }

    return LightBar{
        color,
        p1,
        p2,
        center,
        length,
        width,
        static_cast<float>(line_angle_deg),
        static_cast<float>(area)};
}

} // namespace

LightBarFilter::LightBarFilter(LightBarFilterParams params) : params_(params)
{
}

LightBarFilterResult LightBarFilter::filter(
    const cv::Mat& frame,
    const ArmorPreprocessResult& preprocess) const
{
    LightBarFilterResult result;
    result.candidates.reserve(preprocess.candidates.size());

    for (const ContourCandidate& geom : preprocess.candidates)
    {
        const std::vector<cv::Point>& contour = geom.contour;
        const cv::RotatedRect& rect = geom.rect;
        const cv::Vec4f& line = geom.center_line;
        const double area = geom.area;
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

        if (!inRange(area, params_.min_area, params_.max_area) ||
            !inRange(aspect_ratio, params_.min_aspect_ratio, params_.max_aspect_ratio) ||
            !inRange(line_angle_deg, params_.min_line_angle_deg, params_.max_line_angle_deg) ||
            !inRange(fill_ratio, params_.min_fill_ratio, params_.max_fill_ratio))
        {
            continue;
        }

        const LightColor color = detectLightColor(frame, contour);
        result.candidates.emplace_back(makeLightBar(rect, line_angle_deg, color, area));
    }

    return result;
}

} // namespace auto_aim
