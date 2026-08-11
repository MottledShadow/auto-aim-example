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

std::size_t drawArmorMetrics(
    cv::Mat& vis,
    const std::vector<LightBar>& light_bars,
    const LightBarMatcherParams& params)
{
    //每对装甲用调色板里一个颜色画框+编号，参数集中列到左上角图例；靠颜色+编号双重对应
    const std::vector<cv::Scalar> palette = {
        {0, 255, 255}, {255, 128, 0}, {0, 165, 255}, {255, 0, 255},
        {0, 255, 0}, {255, 255, 0}, {200, 0, 255}, {255, 0, 0},
    };
    std::vector<std::string> legend;        // 左上角每行文字
    std::vector<cv::Scalar> legend_colors;  // 每行颜色，与对应装甲框同色

    std::size_t accepted = 0;
    std::size_t drawn = 0;  // 已画标注的同色对序号
    //双重循环两两配对（与 matchArmors 一致）
    for (std::size_t i = 0; i < light_bars.size(); ++i)
    {
        for (std::size_t j = i + 1; j < light_bars.size(); ++j)
        {
            //按中心 x 归一左右，令 left 在左
            std::size_t left_index = i;
            std::size_t right_index = j;
            if (light_bars[right_index].center.x < light_bars[left_index].center.x)
            {
                std::swap(left_index, right_index);
            }
            const LightBar& left = light_bars[left_index];
            const LightBar& right = light_bars[right_index];

            //只画同色对：颜色相同且非 Unknown，其余不画
            if (left.color == LightColor::Unknown || left.color != right.color)
            {
                continue;
            }

            //长度比：较长/较短，短灯条退化跳过
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

            //逐项判断是否在范围内（与 matchArmors 的五项几何判据一致）
            const bool length_ok = length_ratio <= params.max_light_length_ratio;
            const bool angle_ok = angle_diff <= params.max_light_angle_diff_deg;
            const bool y_ok = center_y_diff <= params.max_light_center_y_diff;
            const bool dist_ok = center_distance_ratio >= params.min_center_distance_ratio &&
                                 center_distance_ratio <= params.max_center_distance_ratio;
            const bool metrics_ok = length_ok && angle_ok && y_ok && dist_ok;

            //大小分类：中心距比 ≥ 阈值判大装甲
            const ArmorType type = (center_distance_ratio >= params.large_armor_min_center_distance_ratio)
                ? ArmorType::Large
                : ArmorType::Small;

            //遮挡判据：装甲四边形内若夹着其它灯条则被拒（同 matchArmors）
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

            //整体接受 = 五项几何量全 ok 且无遮挡（与 matchArmors 的接受条件一致，故计数可与 matchArmors().size() 自检）
            const bool pass = metrics_ok && !has_light_between;
            if (pass)
            {
                ++accepted;
            }

            const std::size_t slot = drawn;
            ++drawn;
            const cv::Scalar color = palette[slot % palette.size()];

            //画装甲四边形，用本对专属颜色；接受粗线(2)、被拒细线(1) —— 线宽区分接受/拒绝
            for (int e = 0; e < 4; ++e)
            {
                cv::line(vis, region[e], region[(e + 1) % 4], color, pass ? 2 : 1);
            }

            //装甲中心标 #编号（同色）
            const cv::Point center(
                cvRound((left.center.x + right.center.x) * 0.5F),
                cvRound((left.center.y + right.center.y) * 0.5F));
            cv::putText(vis, "#" + std::to_string(slot), center,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

            //攒一行图例：#编号 + ok/NG + 各指标(超范围加!) + 大小 + 遮挡
            std::string row = "#" + std::to_string(slot) + (pass ? " ok " : " NG ");
            row += "LR" + fmt(length_ratio, 2) + (length_ok ? " " : "! ");
            row += "ang" + fmt(angle_diff, 1) + (angle_ok ? " " : "! ");
            row += "dY" + fmt(center_y_diff, 0) + (y_ok ? " " : "! ");
            row += "dist" + fmt(center_distance_ratio, 2) + (dist_ok ? " " : "! ");
            row += (type == ArmorType::Large ? "L" : "S");
            if (has_light_between)
            {
                row += " occ";
            }
            legend.push_back(row);
            legend_colors.push_back(color);
        }
    }

    //左上角铺黑底 + 逐行写图例，行色与对应装甲框一致
    if (!legend.empty())
    {
        const int line_h = 16;
        const int pad = 4;
        int max_w = 0;
        for (const auto& text : legend)
        {
            int baseline = 0;
            const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            max_w = std::max(max_w, size.width);
        }
        const int box_h = pad * 2 + static_cast<int>(legend.size()) * line_h;
        cv::rectangle(vis, cv::Rect(0, 0, max_w + pad * 2, box_h), cv::Scalar(0, 0, 0), cv::FILLED);
        for (std::size_t r = 0; r < legend.size(); ++r)
        {
            cv::putText(vis, legend[r], cv::Point(pad, pad + static_cast<int>(r) * line_h + 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, legend_colors[r], 1);
        }
    }

    return accepted;
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
