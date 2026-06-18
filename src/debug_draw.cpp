#include "debug_draw.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

std::vector<cv::Point2f> lightBarBoxPoints(const LightBar& light)
{
    const cv::Point2f axis = light.bottom - light.top;
    const float length = std::hypot(axis.x, axis.y);
    if (length <= 1e-6F || light.width <= 0.0F)
    {
        return {};
    }

    const cv::Point2f normal(-axis.y / length, axis.x / length);
    const cv::Point2f half_width = normal * (light.width * 0.5F);
    return {
        light.top - half_width,
        light.top + half_width,
        light.bottom + half_width,
        light.bottom - half_width,
    };
}

void drawFilteredLightBarBoxes(
    cv::Mat& image,
    const std::vector<LightBar>& light_bars)
{
    for (const LightBar& light : light_bars)
    {
        cv::Scalar color(0, 255, 255);
        if (light.color == LightColor::Red)
        {
            color = cv::Scalar(0, 0, 255);
        }
        else if (light.color == LightColor::Blue)
        {
            color = cv::Scalar(255, 0, 0);
        }

        const std::vector<cv::Point2f> points = lightBarBoxPoints(light);
        if (points.empty())
        {
            continue;
        }

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            cv::line(image, points[i], points[(i + 1) % points.size()], color, 2);
        }

        std::ostringstream os;
        os << "A=" << std::fixed << std::setprecision(0) << light.area;
        const std::string text = os.str();
        int baseline = 0;
        const cv::Size text_size =
            cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);

        float min_x = points.front().x;
        float min_y = points.front().y;
        for (const auto& point : points)
        {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
        }

        const int max_x = std::max(0, image.cols - text_size.width - 4);
        int x = std::max(0, std::min(max_x, cvRound(min_x)));
        int y = cvRound(min_y) - 4;
        if (y - text_size.height - baseline < 0)
        {
            y = cvRound(min_y) + text_size.height + baseline + 4;
        }
        y = std::max(text_size.height + baseline, std::min(image.rows - 2, y));

        const cv::Point anchor(x, y);
        const cv::Point top_left(anchor.x - 2, anchor.y - text_size.height - baseline - 2);
        const cv::Point bottom_right(anchor.x + text_size.width + 2, anchor.y + baseline + 2);
        cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 0, 0), -1);
        cv::putText(image, text, anchor, cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
    }
}

void drawCandidateCenterLines(
    cv::Mat& image,
    const std::vector<ContourCandidate>& candidates)
{
    for (const ContourCandidate& candidate : candidates)
    {
        const cv::Vec4f& line = candidate.center_line;
        const cv::RotatedRect& rect = candidate.rect;
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

cv::Point toImagePoint(const cv::Point2f& point)
{
    return cv::Point(cvRound(point.x), cvRound(point.y));
}

void drawArmors(cv::Mat& image, const std::vector<Armor>& armors)
{
    for (const Armor& armor : armors)
    {
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

    drawFilteredLightBarBoxes(preview, light_bars.candidates);
    drawCandidateCenterLines(preview, preprocess.candidates);
    drawArmors(preview, armors.candidates);
    return preview;
}

} // namespace auto_aim
