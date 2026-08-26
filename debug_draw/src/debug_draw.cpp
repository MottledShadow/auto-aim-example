#include "debug_draw.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace auto_aim::debug_draw
{

//与 lightbar_detector.cpp 里 filterLightBars 一致的退化轮廓判定阈值
constexpr double kEpsilon = 1e-6;

//画框/图例的调色板：灯条层逐亮斑取一色，配对层逐装甲对取一色
static const std::vector<cv::Scalar> kPalette = {
    {0, 255, 255}, {255, 128, 0}, {0, 165, 255}, {255, 0, 255},
    {0, 255, 0}, {255, 255, 0}, {200, 0, 255}, {255, 0, 0},
};

//把一个 double 转成固定小数位的短字符串，用来标注
static std::string format(double value, int decimals)
{
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed << value;
    return oss.str();
}

//图例的一行：行首一个 swatch 色块（与图中框同色作对应）+ 若干带各自颜色的文本分段
struct LegendRow
{
    cv::Scalar swatch;
    std::vector<std::pair<std::string, cv::Scalar>> segments;
};

//左上角铺黑底逐行画图例：每行先画 swatch 色块，再横排写各分段（颜色各自独立）
static void renderLegend(cv::Mat& vis, const std::vector<LegendRow>& legend)
{
    if (legend.empty())
    {
        return;
    }
    const int lineH = 16;
    const int pad = 4;
    const int swatchW = 12;

    //每行宽 = 色块 + 各分段文本宽之和，取所有行最大
    int maxW = 0;
    for (const auto& row : legend)
    {
        int rowW = swatchW + 4;
        for (const auto& seg : row.segments)
        {
            int baseline = 0;
            const cv::Size size = cv::getTextSize(seg.first, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            rowW += size.width;
        }
        maxW = std::max(maxW, rowW);
    }

    const int boxH = pad * 2 + static_cast<int>(legend.size()) * lineH;
    cv::rectangle(vis, cv::Rect(0, 0, maxW + pad * 2, boxH), cv::Scalar(0, 0, 0), cv::FILLED);

    //逐行：先画色块，再逐段接写
    for (std::size_t r = 0; r < legend.size(); ++r)
    {
        const int y = pad + static_cast<int>(r) * lineH + 12;
        cv::rectangle(vis, cv::Rect(pad, y - 10, swatchW, 10), legend[r].swatch, cv::FILLED);
        int x = pad + swatchW + 4;
        for (const auto& seg : legend[r].segments)
        {
            cv::putText(vis, seg.first, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4, seg.second, 1);
            int baseline = 0;
            const cv::Size size = cv::getTextSize(seg.first, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            x += size.width;
        }
    }
}

cv::Mat sideBySide(const cv::Mat& original, const cv::Mat& binary, int threshold)
{
    //二值图转 BGR 后与原图左右拼接，各自标题；binary 侧标注灰度阈值
    cv::Mat binaryBgr;
    cv::cvtColor(binary, binaryBgr, cv::COLOR_GRAY2BGR);

    cv::Mat combined;
    cv::hconcat(original, binaryBgr, combined);

    const cv::Scalar green(0, 255, 0);
    cv::putText(combined, "original", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
    cv::putText(combined, "binary thresh=" + std::to_string(threshold),
                cv::Point(original.cols + 10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, green, 2);
    return combined;
}

void drawCandidates(cv::Mat& vis, const std::vector<detector::ContourCandidate>& candidates)
{
    for (const auto& candidate : candidates)
    {
        //候选轮廓（绿色）
        const std::vector<std::vector<cv::Point>> oneContour{candidate.contour};
        cv::drawContours(vis, oneContour, -1, cv::Scalar(0, 255, 0), 1);

        //最小外接矩形（黄色）
        cv::Point2f corners[4];
        candidate.rect.points(corners);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(vis, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
        }

        //fitLine 中心线（红色），沿方向向量往两边延伸
        const float vx = candidate.centerLine[0];
        const float vy = candidate.centerLine[1];
        const float x0 = candidate.centerLine[2];
        const float y0 = candidate.centerLine[3];
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
    const std::vector<detector::ContourCandidate>& candidates,
    const detector::LightBarFilterParams& params)
{
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar red(0, 0, 255);

    std::vector<LegendRow> legend;
    std::size_t passed = 0;
    std::size_t drawn = 0;  // 已画的亮斑序号，用于取调色板颜色（图上不显示序号）
    for (const auto& candidate : candidates)
    {
        const cv::RotatedRect& rect = candidate.rect;
        const double area = candidate.area;

        //取最小外接矩形的长边短边，退化轮廓跳过（与生产一致）
        const double longSide = std::max(rect.size.width, rect.size.height);
        const double shortSide = std::min(rect.size.width, rect.size.height);
        if (shortSide <= kEpsilon)
        {
            continue;
        }
        const double rectArea = longSide * shortSide;
        if (rectArea <= kEpsilon)
        {
            continue;
        }

        //长宽比
        const double aspectRatio = longSide / shortSide;

        //中心线与竖直方向的夹角（度），方向向量模为 0 时记 90 度
        const double vx = candidate.centerLine[0];
        const double vy = candidate.centerLine[1];
        const double norm = std::hypot(vx, vy);
        double lineAngleDeg = 90.0;
        if (norm > kEpsilon)
        {
            lineAngleDeg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
        }

        //轮廓面积占外接矩形的比例
        const double fillRatio = std::clamp(area / rectArea, 0.0, 1.0);

        //逐项判断是否在范围内
        const bool areaOk = area >= params.minArea && area <= params.maxArea;
        const bool aspectOk = aspectRatio >= params.minAspectRatio && aspectRatio <= params.maxAspectRatio;
        const bool angleOk = lineAngleDeg >= params.minLineAngleDeg && lineAngleDeg <= params.maxLineAngleDeg;
        const bool fillOk = fillRatio >= params.minFillRatio && fillRatio <= params.maxFillRatio;
        const bool pass = areaOk && aspectOk && angleOk && fillOk;
        if (pass)
        {
            ++passed;
        }

        //本亮斑专属颜色
        const cv::Scalar color = kPalette[drawn % kPalette.size()];
        ++drawn;

        //轮廓（本色、细线）
        cv::drawContours(vis, std::vector<std::vector<cv::Point>>{candidate.contour}, -1, color, 1);

        //最小外接矩形（本色、细线，方便看角点）
        cv::Point2f corners[4];
        rect.points(corners);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(vis, corners[i], corners[(i + 1) % 4], color, 1);
        }

        //fitLine 中心线（本色、细线），沿方向向量往两边延伸
        const float x0 = candidate.centerLine[2];
        const float y0 = candidate.centerLine[3];
        const float ext = 30.0F;
        const cv::Point p1(cvRound(x0 - vx * ext), cvRound(y0 - vy * ext));
        const cv::Point p2(cvRound(x0 + vx * ext), cvRound(y0 + vy * ext));
        cv::line(vis, p1, p2, color, 1);

        //攒一行图例：行首色块与本亮斑框同色，四项各自 ok 绿/超范围红
        LegendRow row;
        row.swatch = color;
        row.segments.emplace_back(std::string(pass ? "ok " : "NG "), color);
        row.segments.emplace_back("A=" + format(area, 0) + " ", areaOk ? green : red);
        row.segments.emplace_back("AR=" + format(aspectRatio, 2) + " ", aspectOk ? green : red);
        row.segments.emplace_back("ang=" + format(lineAngleDeg, 1) + " ", angleOk ? green : red);
        row.segments.emplace_back("fill=" + format(fillRatio, 2), fillOk ? green : red);
        legend.push_back(row);
    }

    //左上角铺图例
    renderLegend(vis, legend);

    return passed;
}

std::size_t drawArmorMetrics(
    cv::Mat& vis,
    const std::vector<detector::LightBar>& lightBars,
    const detector::LightBarMatcherParams& params)
{
    //在范围绿、超范围红
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar white(255, 255, 255);

    //先给每根灯条在中心标索引（白字），图例的 i+j 就指这些索引
    for (std::size_t i = 0; i < lightBars.size(); ++i)
    {
        cv::putText(vis, std::to_string(i),
                    cv::Point(cvRound(lightBars[i].center.x), cvRound(lightBars[i].center.y)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, white, 2);
    }

    //左上角图例：每行行首色块与本对装甲框同色，"i+j" 起头后跟各指标 ok 绿/超范围红
    std::vector<LegendRow> legend;

    std::size_t accepted = 0;
    std::size_t drawn = 0;  // 已画标注的同色对序号，用于取调色板颜色
    //双重循环两两配对（与 matchArmors 一致）
    for (std::size_t i = 0; i < lightBars.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lightBars.size(); ++j)
        {
            //按中心 x 归一左右，令 left 在左
            std::size_t leftIndex = i;
            std::size_t rightIndex = j;
            if (lightBars[rightIndex].center.x < lightBars[leftIndex].center.x)
            {
                std::swap(leftIndex, rightIndex);
            }
            const detector::LightBar& left = lightBars[leftIndex];
            const detector::LightBar& right = lightBars[rightIndex];

            //只画同色对：颜色相同且非 Unknown，其余不画
            if (left.color == detector::LightColor::Unknown || left.color != right.color)
            {
                continue;
            }

            //长度比：较长/较短，短灯条退化跳过
            const double minLength = std::min(left.length, right.length);
            if (minLength <= kEpsilon)
            {
                continue;
            }
            const double lengthRatio = std::max(left.length, right.length) / minLength;

            //角度差、中心 y 差
            const double angleDiff = std::abs(left.angle - right.angle);
            const double centerYDiff = std::abs(left.center.y - right.center.y);

            //中心距比：两中心欧氏距 / 平均灯条长，平均长退化跳过
            const double averageHeight = (left.length + right.length) * 0.5;
            if (averageHeight <= kEpsilon)
            {
                continue;
            }
            const double centerDistanceRatio =
                std::hypot(left.center.x - right.center.x, left.center.y - right.center.y) /
                averageHeight;

            //逐项判断是否在范围内（与 matchArmors 的五项几何判据一致）
            const bool lengthOk = lengthRatio <= params.maxLightLengthRatio;
            const bool angleOk = angleDiff <= params.maxLightAngleDiffDeg;
            const bool yOk = centerYDiff <= params.maxLightCenterYDiff;
            const bool distOk = centerDistanceRatio >= params.minCenterDistanceRatio &&
                                 centerDistanceRatio <= params.maxCenterDistanceRatio;
            const bool metricsOk = lengthOk && angleOk && yOk && distOk;

            //大小分类：中心距比 ≥ 阈值判大装甲
            const detector::ArmorType type = (centerDistanceRatio >= params.largeArmorMinCenterDistanceRatio)
                ? detector::ArmorType::Large
                : detector::ArmorType::Small;

            //遮挡判据：装甲四边形内若夹着其它灯条则被拒（同 matchArmors）
            const std::vector<cv::Point2f> region = {
                left.top,
                right.top,
                right.bottom,
                left.bottom,
            };
            bool hasLightBetween = false;
            for (std::size_t k = 0; k < lightBars.size(); ++k)
            {
                if (k == leftIndex || k == rightIndex)
                {
                    continue;
                }
                const detector::LightBar& other = lightBars[k];
                if (cv::pointPolygonTest(region, other.top, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.bottom, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.center, false) >= 0.0)
                {
                    hasLightBetween = true;
                    break;
                }
            }

            //整体接受 = 五项几何量全 ok 且无遮挡（与 matchArmors 的接受条件一致，故计数可与 matchArmors().size() 自检）
            const bool pass = metricsOk && !hasLightBetween;
            if (pass)
            {
                ++accepted;
            }

            const cv::Scalar color = kPalette[drawn % kPalette.size()];
            ++drawn;

            //画装甲四边形，用本对专属颜色；接受粗线(2)、被拒细线(1) —— 线宽区分接受/拒绝
            for (int e = 0; e < 4; ++e)
            {
                cv::line(vis, region[e], region[(e + 1) % 4], color, pass ? 2 : 1);
            }

            //攒一行图例：行首色块与本对框同色，"i+j" 指两根灯条索引，各指标 ok 绿/超范围红，遮挡标红
            LegendRow row;
            row.swatch = color;
            row.segments.emplace_back(
                std::to_string(leftIndex) + "+" + std::to_string(rightIndex) +
                    (pass ? " ok " : " NG ") + (type == detector::ArmorType::Large ? "L " : "S "),
                color);
            row.segments.emplace_back("LR" + format(lengthRatio, 2) + " ", lengthOk ? green : red);
            row.segments.emplace_back("ang" + format(angleDiff, 1) + " ", angleOk ? green : red);
            row.segments.emplace_back("dY" + format(centerYDiff, 0) + " ", yOk ? green : red);
            row.segments.emplace_back("dist" + format(centerDistanceRatio, 2), distOk ? green : red);
            if (hasLightBetween)
            {
                row.segments.emplace_back(" occ", red);
            }
            legend.push_back(row);
        }
    }

    //左上角铺图例
    renderLegend(vis, legend);

    return accepted;
}

void fitToScreen(cv::Mat& image, int screenW, int screenH)
{
    //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比），只缩不放
    const double scale = std::min(
        static_cast<double>(screenW) / image.cols,
        static_cast<double>(screenH) / image.rows);
    if (scale < 1.0)
    {
        cv::resize(image, image, cv::Size(), scale, scale, cv::INTER_AREA);
    }
}

} // namespace auto_aim::debug_draw
