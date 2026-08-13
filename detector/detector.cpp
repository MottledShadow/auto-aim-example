#include "detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

PreprocessResult Detector::preprocess(const cv::Mat& frame) const
{
    PreprocessResult result;

    //灰度二值化：转灰度后直接阈值化
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(
        gray,
        result.binary,
        preprocess_params.binary_threshold,
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
            cv::fitLine(contour, center_line, cv::DIST_L2, 0, 0.01, 0.01);
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

std::vector<LightBar> Detector::filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& pre) const
{
    std::vector<LightBar> result;
    result.reserve(pre.candidates.size());

    for (const ContourCandidate& geom : pre.candidates)
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
        if (area < filter_params.min_area || area > filter_params.max_area ||
            aspect_ratio < filter_params.min_aspect_ratio || aspect_ratio > filter_params.max_aspect_ratio ||
            line_angle_deg < filter_params.min_line_angle_deg || line_angle_deg > filter_params.max_line_angle_deg ||
            fill_ratio < filter_params.min_fill_ratio || fill_ratio > filter_params.max_fill_ratio)
        {
            continue;
        }

        //判定灯条颜色：灰度二值图不含颜色，取轮廓内 BGR 均值判红蓝
        LightColor color = LightColor::Unknown;
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{geom.contour}, 0, cv::Scalar(255), cv::FILLED);
        const cv::Scalar mean = cv::mean(frame, mask);
        if (mean[2] > mean[0])
        {
            color = LightColor::Red;
        }
        else if (mean[0] > mean[2])
        {
            color = LightColor::Blue;
        }

        //颜色不是目标灯条颜色的直接舍弃
        if (color != filter_params.target_color)
        {
            continue;
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
            static_cast<float>(line_angle_deg)});
    }

    return result;
}

std::vector<Armor> Detector::matchArmors(const std::vector<LightBar>& light_bars) const
{
    std::vector<Armor> result;

    //双重循环两两配对全部灯条
    for (std::size_t i = 0; i < light_bars.size(); ++i)
    {
        for (std::size_t j = i + 1; j < light_bars.size(); ++j)
        {
            //按中心 x 归一左右，令 left 在左（x 较小）
            std::size_t left_index = i;
            std::size_t right_index = j;
            if (light_bars[right_index].center.x < light_bars[left_index].center.x)
            {
                std::swap(left_index, right_index);
            }
            const LightBar& left = light_bars[left_index];
            const LightBar& right = light_bars[right_index];

            //长度比：较长/较短，短灯条退化（≈0）跳过
            const double min_length = std::min(left.length, right.length);
            if (min_length <= kEpsilon)
            {
                continue;
            }
            const double length_ratio = std::max(left.length, right.length) / min_length;

            //角度差、中心 y 差
            const double angle_diff = std::abs(left.angle - right.angle);
            const double center_y_diff = std::abs(left.center.y - right.center.y);

            //中心距比：两中心欧氏距 / 平均灯条长，平均长退化跳过
            const double average_height = (left.length + right.length) * 0.5;
            if (average_height <= kEpsilon)
            {
                continue;
            }
            const double center_distance_ratio =
                std::hypot(left.center.x - right.center.x, left.center.y - right.center.y) /
                average_height;

            //五项几何判据逐一范围筛选，任一超范围就跳过
            if (length_ratio > matcher_params.max_light_length_ratio ||
                angle_diff > matcher_params.max_light_angle_diff_deg ||
                center_y_diff > matcher_params.max_light_center_y_diff ||
                center_distance_ratio < matcher_params.min_center_distance_ratio ||
                center_distance_ratio > matcher_params.max_center_distance_ratio)
            {
                continue;
            }

            //遮挡判据：两灯条上下端点围成的装甲四边形内若夹着其它灯条，跳过该对
            const std::vector<cv::Point2f> region = {
                left.top,
                right.top,
                right.bottom,
                left.bottom,
            };
            bool has_light_between = false;
            for (std::size_t k = 0; k < light_bars.size(); ++k)
            {
                if (k == left_index || k == right_index)
                {
                    continue;
                }
                const LightBar& other = light_bars[k];
                if (cv::pointPolygonTest(region, other.top, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.bottom, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.center, false) >= 0.0)
                {
                    has_light_between = true;
                    break;
                }
            }
            if (has_light_between)
            {
                continue;
            }

            //大小分类：中心距比 ≥ 阈值判大装甲，否则小装甲
            const ArmorType type = (center_distance_ratio >= matcher_params.large_armor_min_center_distance_ratio)
                ? ArmorType::Large
                : ArmorType::Small;

            //组装装甲板，中心取两灯条中心的中点（number/confidence 留待分类阶段填）
            Armor armor;
            armor.left_light = left;
            armor.right_light = right;
            armor.center = (left.center + right.center) * 0.5F;
            armor.type = type;
            result.push_back(armor);
        }
    }

    return result;
}

} // namespace auto_aim
