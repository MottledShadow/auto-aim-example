#include "debug_draw.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

//与 detector.cpp 里 filterLightBars 一致的退化轮廓判定阈值
constexpr double kEpsilon = 1e-6;

//把一个 double 转成固定小数位的短字符串，用来标注
static std::string fmt(double value, int decimals)
{
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed << value;
    return oss.str();
}

void drawCandidates(cv::Mat& vis, const std::vector<ContourCandidate>& candidates)
{
    for (const auto& candidate : candidates)
    {
        //候选轮廓（绿色）
        const std::vector<std::vector<cv::Point>> one_contour{candidate.contour};
        cv::drawContours(vis, one_contour, -1, cv::Scalar(0, 255, 0), 1);

        //最小外接矩形（黄色）
        cv::Point2f corners[4];
        candidate.rect.points(corners);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(vis, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
        }

        //fitLine 中心线（红色），沿方向向量往两边延伸
        const float vx = candidate.center_line[0];
        const float vy = candidate.center_line[1];
        const float x0 = candidate.center_line[2];
        const float y0 = candidate.center_line[3];
        const float length = 30.0F;
        const cv::Point p1(cvRound(x0 - vx * length), cvRound(y0 - vy * length));
        const cv::Point p2(cvRound(x0 + vx * length), cvRound(y0 + vy * length));
        cv::line(vis, p1, p2, cv::Scalar(0, 0, 255), 1);

        //面积数字
        cv::putText(
            vis,
            "A=" + std::to_string(static_cast<int>(candidate.area)),
            candidate.rect.center,
            cv::FONT_HERSHEY_SIMPLEX,
            0.4,
            cv::Scalar(0, 0, 255),
            1);
    }
}

std::size_t drawLightBarMetrics(
    cv::Mat& vis,
    const std::vector<ContourCandidate>& candidates,
    const LightBarFilterParams& params)
{
    std::size_t passed = 0;
    for (const auto& candidate : candidates)
    {
        const cv::RotatedRect& rect = candidate.rect;
        const double area = candidate.area;

        //取最小外接矩形的长边短边，退化轮廓跳过（与生产一致）
        const double long_side = std::max(rect.size.width, rect.size.height);
        const double short_side = std::min(rect.size.width, rect.size.height);
        if (short_side <= kEpsilon)
        {
            continue;
        }
        const double rect_area = long_side * short_side;
        if (rect_area <= kEpsilon)
        {
            continue;
        }

        //长宽比
        const double aspect_ratio = long_side / short_side;

        //中心线与竖直方向的夹角（度），方向向量模为 0 时记 90 度
        const double vx = candidate.center_line[0];
        const double vy = candidate.center_line[1];
        const double norm = std::hypot(vx, vy);
        double line_angle_deg = 90.0;
        if (norm > kEpsilon)
        {
            line_angle_deg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
        }

        //轮廓面积占外接矩形的比例
        const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

        //逐项判断是否在范围内
        const bool area_ok = area >= params.min_area && area <= params.max_area;
        const bool aspect_ok = aspect_ratio >= params.min_aspect_ratio && aspect_ratio <= params.max_aspect_ratio;
        const bool angle_ok = line_angle_deg >= params.min_line_angle_deg && line_angle_deg <= params.max_line_angle_deg;
        const bool fill_ok = fill_ratio >= params.min_fill_ratio && fill_ratio <= params.max_fill_ratio;
        const bool pass = area_ok && aspect_ok && angle_ok && fill_ok;
        if (pass)
        {
            ++passed;
        }

        //在范围绿、超范围红
        const cv::Scalar green(0, 255, 0);
        const cv::Scalar red(0, 0, 255);

        //画最小外接矩形，整体通过绿、被拒红
        cv::Point2f corners[4];
        rect.points(corners);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(vis, corners[i], corners[(i + 1) % 4], pass ? green : red, 1);
        }

        //在轮廓旁边分四行标注四项值，每行按该项是否在范围单独着色
        const cv::Point anchor(cvRound(rect.center.x), cvRound(rect.center.y));
        cv::putText(vis, "A=" + fmt(area, 0), anchor + cv::Point(0, 0),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, area_ok ? green : red, 1);
        cv::putText(vis, "AR=" + fmt(aspect_ratio, 2), anchor + cv::Point(0, 14),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, aspect_ok ? green : red, 1);
        cv::putText(vis, "ang=" + fmt(line_angle_deg, 1), anchor + cv::Point(0, 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, angle_ok ? green : red, 1);
        cv::putText(vis, "fill=" + fmt(fill_ratio, 2), anchor + cv::Point(0, 42),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, fill_ok ? green : red, 1);
    }

    return passed;
}

void fitToScreen(cv::Mat& image, int screen_w, int screen_h)
{
    //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比），只缩不放
    const double scale = std::min(
        static_cast<double>(screen_w) / image.cols,
        static_cast<double>(screen_h) / image.rows);
    if (scale < 1.0)
    {
        cv::resize(image, image, cv::Size(), scale, scale, cv::INTER_AREA);
    }
}

} // namespace auto_aim
