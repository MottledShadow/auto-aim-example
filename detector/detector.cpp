#include "detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

PreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params)
{
    PreprocessResult result;

    //转为灰度图
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    //二值化
    cv::threshold(
        gray,
        result.binary,
        params.binary_threshold,
        255,
        cv::THRESH_BINARY);

    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contour_input = result.binary.clone();
    cv::findContours(
        contour_input,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    //计算轮廓的最小外接矩形和中心线并存储到结果中
    result.candidates.reserve(contours.size());
    for (const auto& contour : contours)
    {
        if (contour.size() >= 2)
        {
            cv::Vec4f center_line;
            cv::fitLine(contour, center_line, cv::DIST_L2, 0, 0.01, 0.01);  //后面需要考虑是否要往后面过程移动
            result.candidates.push_back(ContourCandidate{
                contour,
                cv::minAreaRect(contour),
                center_line,
                std::abs(cv::contourArea(contour)),
            });
        }
    }

    return result;
}

constexpr double kEpsilon = 1e-6;

std::vector<LightBar> filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& preprocess,
    const LightBarFilterParams& params)
{
    std::vector<LightBar> result;
    result.reserve(preprocess.candidates.size());

    for (const ContourCandidate& geom : preprocess.candidates)
    {
        const cv::RotatedRect& rect = geom.rect;
        const double area = geom.area;

        //取最小外接矩形的长边短边
        const double long_side = std::max(rect.size.width, rect.size.height);
        const double short_side = std::min(rect.size.width, rect.size.height);

        //短边或矩形面积为 0 的退化轮廓跳过
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
        const double vx = geom.center_line[0];
        const double vy = geom.center_line[1];
        const double norm = std::hypot(vx, vy);
        double line_angle_deg = 90.0;
        if (norm > kEpsilon)
        {
            line_angle_deg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
        }

        //轮廓面积占外接矩形的比例
        const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

        //面积、长宽比、角度、填充比逐项范围筛选，任一超范围就跳过
        if (area < params.min_area || area > params.max_area ||
            aspect_ratio < params.min_aspect_ratio || aspect_ratio > params.max_aspect_ratio ||
            line_angle_deg < params.min_line_angle_deg || line_angle_deg > params.max_line_angle_deg ||
            fill_ratio < params.min_fill_ratio || fill_ratio > params.max_fill_ratio)
        {
            continue;
        }

        //在轮廓区域内取 BGR 均值判断红蓝
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{geom.contour}, 0, cv::Scalar(255), cv::FILLED);
        const cv::Scalar mean = cv::mean(frame, mask);
        LightColor color = LightColor::Unknown;
        if (mean[2] > mean[0])
        {
            color = LightColor::Red;
        }
        else if (mean[0] > mean[2])
        {
            color = LightColor::Blue;
        }

        //由外接矩形顶点求灯条上下两端点：取较短边方向的两条边中点，按 y 排序令 top 在上
        cv::Point2f vertices[4];
        rect.points(vertices);
        cv::Point2f top;
        cv::Point2f bottom;
        const double edge01 = std::hypot(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
        const double edge12 = std::hypot(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);
        if (edge01 <= edge12)
        {
            top = (vertices[0] + vertices[1]) * 0.5F;
            bottom = (vertices[2] + vertices[3]) * 0.5F;
        }
        else
        {
            top = (vertices[1] + vertices[2]) * 0.5F;
            bottom = (vertices[3] + vertices[0]) * 0.5F;
        }
        if (top.y > bottom.y)
        {
            std::swap(top, bottom);
        }

        //存入筛选结果
        result.push_back(LightBar{
            color,
            top,
            bottom,
            rect.center,
            static_cast<float>(long_side),
            static_cast<float>(short_side),
            static_cast<float>(line_angle_deg),
            static_cast<float>(area)});
    }

    return result;
}

} // namespace auto_aim
