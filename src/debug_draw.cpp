#include "debug_draw.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

std::string areaLabel(double area)
{
    std::ostringstream os;
    os << "A=" << std::fixed << std::setprecision(0) << area;
    return os.str();
}

cv::Point candidateLabelAnchor(
    const cv::Mat& image,
    const cv::RotatedRect& rect,
    const cv::Size& text_size,
    int baseline)
{
    cv::Point2f vertices[4];
    rect.points(vertices);

    float min_x = vertices[0].x;
    float min_y = vertices[0].y;
    for (int i = 1; i < 4; ++i)
    {
        min_x = std::min(min_x, vertices[i].x);
        min_y = std::min(min_y, vertices[i].y);
    }

    const int max_x = std::max(0, image.cols - text_size.width - 4);
    int x = std::max(0, std::min(max_x, cvRound(min_x)));
    int y = cvRound(min_y) - 4;

    if (y - text_size.height - baseline < 0)
    {
        y = cvRound(min_y) + text_size.height + baseline + 4;
    }
    y = std::max(text_size.height + baseline, std::min(image.rows - 2, y));
    return cv::Point(x, y);
}

void drawTextWithBackground(
    cv::Mat& image,
    const std::string& text,
    const cv::Point& anchor,
    const cv::Size& text_size,
    int baseline,
    const cv::Scalar& color)
{
    const cv::Point top_left(anchor.x - 2, anchor.y - text_size.height - baseline - 2);
    const cv::Point bottom_right(anchor.x + text_size.width + 2, anchor.y + baseline + 2);
    cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 0, 0), -1);
    cv::putText(image, text, anchor, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
}

void drawCandidateRects(
    cv::Mat& image,
    const std::vector<cv::RotatedRect>& rects,
    const std::vector<double>& areas)
{
    const cv::Scalar color(0, 255, 0);
    for (std::size_t i = 0; i < rects.size(); ++i)
    {
        const auto& rect = rects[i];
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
        }

        if (i < areas.size())
        {
            const std::string text = areaLabel(areas[i]);
            int baseline = 0;
            const cv::Size text_size =
                cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
            drawTextWithBackground(
                image,
                text,
                candidateLabelAnchor(image, rect, text_size, baseline),
                text_size,
                baseline,
                color);
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

    drawCandidateRects(preview, preprocess.candidate_rects, preprocess.candidate_areas);
    drawCandidateCenterLines(preview, preprocess.candidate_rects, preprocess.candidate_center_lines);
    drawLightBars(preview, light_bars.candidates);
    drawArmors(preview, armors.candidates);
    return preview;
}

} // namespace auto_aim
