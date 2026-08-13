#include "geometry_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

PreprocessResult GeometryDetector::preprocess(const cv::Mat& frame) const
{
    PreprocessResult result;

    //灰度二值化：转灰度后直接阈值化
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(
        gray,
        result.binary,
        preprocessParams.binaryThreshold,
        255,
        cv::THRESH_BINARY);

    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contourInput = result.binary.clone();
    cv::findContours(
        contourInput,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    //计算轮廓的最小外接矩形和中心线并存储到结果中
    result.candidates.reserve(contours.size());
    for (const auto& contour : contours)
    {
        if (contour.size() >= 2)
        {
            cv::Vec4f centerLine;
            cv::fitLine(contour, centerLine, cv::DIST_L2, 0, 0.01, 0.01);
            result.candidates.push_back(ContourCandidate{
                contour,
                cv::minAreaRect(contour),
                centerLine,
                std::abs(cv::contourArea(contour)),
            });
        }
    }

    return result;
}

constexpr double kEpsilon = 1e-6;

std::vector<LightBar> GeometryDetector::filterLightBars(
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
        const double longSide = std::max(rect.size.width, rect.size.height);
        const double shortSide = std::min(rect.size.width, rect.size.height);

        //短边或矩形面积为 0 的退化轮廓跳过
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
        const double vx = geom.centerLine[0];
        const double vy = geom.centerLine[1];
        const double norm = std::hypot(vx, vy);
        double lineAngleDeg = 90.0;
        if (norm > kEpsilon)
        {
            lineAngleDeg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
        }

        //轮廓面积占外接矩形的比例
        const double fillRatio = std::clamp(area / rectArea, 0.0, 1.0);

        //面积、长宽比、角度、填充比逐项范围筛选，任一超范围就跳过
        if (area < filterParams.minArea || area > filterParams.maxArea ||
            aspectRatio < filterParams.minAspectRatio || aspectRatio > filterParams.maxAspectRatio ||
            lineAngleDeg < filterParams.minLineAngleDeg || lineAngleDeg > filterParams.maxLineAngleDeg ||
            fillRatio < filterParams.minFillRatio || fillRatio > filterParams.maxFillRatio)
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
        if (color != filterParams.targetColor)
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
            static_cast<float>(longSide),
            static_cast<float>(lineAngleDeg)});
    }

    return result;
}

std::vector<Armor> GeometryDetector::matchArmors(const std::vector<LightBar>& lightBars) const
{
    std::vector<Armor> result;

    //双重循环两两配对全部灯条
    for (std::size_t i = 0; i < lightBars.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lightBars.size(); ++j)
        {
            //按中心 x 归一左右，令 left 在左（x 较小）
            std::size_t leftIndex = i;
            std::size_t rightIndex = j;
            if (lightBars[rightIndex].center.x < lightBars[leftIndex].center.x)
            {
                std::swap(leftIndex, rightIndex);
            }
            const LightBar& left = lightBars[leftIndex];
            const LightBar& right = lightBars[rightIndex];

            //长度比：较长/较短，短灯条退化（≈0）跳过
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

            //五项几何判据逐一范围筛选，任一超范围就跳过
            if (lengthRatio > matcherParams.maxLightLengthRatio ||
                angleDiff > matcherParams.maxLightAngleDiffDeg ||
                centerYDiff > matcherParams.maxLightCenterYDiff ||
                centerDistanceRatio < matcherParams.minCenterDistanceRatio ||
                centerDistanceRatio > matcherParams.maxCenterDistanceRatio)
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
            bool hasLightBetween = false;
            for (std::size_t k = 0; k < lightBars.size(); ++k)
            {
                if (k == leftIndex || k == rightIndex)
                {
                    continue;
                }
                const LightBar& other = lightBars[k];
                if (cv::pointPolygonTest(region, other.top, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.bottom, false) >= 0.0 ||
                    cv::pointPolygonTest(region, other.center, false) >= 0.0)
                {
                    hasLightBetween = true;
                    break;
                }
            }
            if (hasLightBetween)
            {
                continue;
            }

            //大小分类：中心距比 ≥ 阈值判大装甲，否则小装甲
            const ArmorType type = (centerDistanceRatio >= matcherParams.largeArmorMinCenterDistanceRatio)
                ? ArmorType::Large
                : ArmorType::Small;

            //组装装甲板，中心取两灯条中心的中点（number/confidence 留待分类阶段填）
            Armor armor;
            armor.leftLight = left;
            armor.rightLight = right;
            armor.center = (left.center + right.center) * 0.5F;
            armor.type = type;
            result.push_back(armor);
        }
    }

    return result;
}

} // namespace auto_aim
