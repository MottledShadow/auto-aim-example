#include "debug_draw.hpp"

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

void drawCandidateRects(cv::Mat& image, const std::vector<cv::RotatedRect>& rects)
{
    for (const auto& rect : rects)
    {
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
}

void drawCandidateCenterLines(
    cv::Mat& image,
    const std::vector<cv::RotatedRect>& rects,
    const std::vector<cv::Vec4f>& center_lines)
{
    const std::size_t count = std::min(rects.size(), center_lines.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        const cv::Vec4f& line = center_lines[i];
        const cv::RotatedRect& rect = rects[i];
        const cv::Point2f direction(line[0], line[1]);
        const cv::Point2f point_on_line(line[2], line[3]);
        const float half_length = std::max(rect.size.width, rect.size.height) * 0.5F;
        cv::line(
            image,
            point_on_line - direction * half_length,
            point_on_line + direction * half_length,
            cv::Scalar(0, 0, 255),
            2);
    }
}

void drawLightBars(cv::Mat& image, const std::vector<LightBarCandidate>& light_bars)
{
    for (const auto& candidate : light_bars)
    {
        const auto& light = candidate.light_bar;
        cv::Scalar color(0, 255, 255);
        if (light.color == LightColor::Red)
        {
            color = cv::Scalar(0, 0, 255);
        }
        else if (light.color == LightColor::Blue)
        {
            color = cv::Scalar(255, 0, 0);
        }

        cv::line(image, light.top, light.bottom, color, 3);
        cv::circle(image, light.center, 3, color, -1);
    }
}

cv::Point toImagePoint(const cv::Point2f& point)
{
    return cv::Point(cvRound(point.x), cvRound(point.y));
}

void drawArmors(cv::Mat& image, const std::vector<ArmorCandidate>& armors)
{
    for (const auto& candidate : armors)
    {
        const auto& armor = candidate.armor;
        std::vector<cv::Point> points{
            toImagePoint(armor.left_light.top),
            toImagePoint(armor.right_light.top),
            toImagePoint(armor.right_light.bottom),
            toImagePoint(armor.left_light.bottom),
        };
        cv::polylines(image, points, true, cv::Scalar(255, 255, 255), 2);
        cv::circle(image, toImagePoint(armor.center), 4, cv::Scalar(255, 255, 255), -1);
    }
}

} // namespace

cv::Mat makeDebugPreview(
    const cv::Mat& frame,
    const ArmorPreprocessResult& preprocess,
    const LightBarFilterResult& light_bars,
    const ArmorMatchResult& armors,
    bool show_binary)
{
    cv::Mat preview;
    if (show_binary)
    {
        cv::cvtColor(preprocess.binary, preview, cv::COLOR_GRAY2BGR);
    }
    else if (frame.channels() == 1)
    {
        cv::cvtColor(frame, preview, cv::COLOR_GRAY2BGR);
    }
    else
    {
        preview = frame.clone();
    }

    drawCandidateRects(preview, preprocess.candidate_rects);
    drawCandidateCenterLines(preview, preprocess.candidate_rects, preprocess.candidate_center_lines);
    drawLightBars(preview, light_bars.candidates);
    drawArmors(preview, armors.candidates);
    return preview;
}

} // namespace auto_aim
