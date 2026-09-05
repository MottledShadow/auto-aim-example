#include "lightbar_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim::detector
{

PreprocessResult LightbarDetector::preprocess(const cv::Mat& frame) const
{
    PreprocessResult result;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(
        gray,
        result.binary,
        binaryThreshold,
        255,
        cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contourInput = result.binary.clone();
    cv::findContours(
        contourInput,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

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

std::vector<LightBar> LightbarDetector::filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& pre) const
{
    std::vector<LightBar> result;
    result.reserve(pre.candidates.size());

    for (const ContourCandidate& geom : pre.candidates)
    {
        const cv::RotatedRect& rect = geom.rect;
        const double area = geom.area;

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

        const double aspectRatio = longSide / shortSide;

        const double vx = geom.centerLine[0];
        const double vy = geom.centerLine[1];
        const double norm = std::hypot(vx, vy);
        double lineAngleDeg = 90.0;
        if (norm > kEpsilon)
        {
            lineAngleDeg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
        }

        const double fillRatio = std::clamp(area / rectArea, 0.0, 1.0);

        if (area < filterParams.minArea || area > filterParams.maxArea ||
            aspectRatio < filterParams.minAspectRatio || aspectRatio > filterParams.maxAspectRatio ||
            lineAngleDeg < filterParams.minLineAngleDeg || lineAngleDeg > filterParams.maxLineAngleDeg ||
            fillRatio < filterParams.minFillRatio || fillRatio > filterParams.maxFillRatio)
        {
            continue;
        }

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

        if (color != filterParams.targetColor)
        {
            continue;
        }

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

std::vector<Armor> LightbarDetector::matchArmors(const std::vector<LightBar>& lightBars) const
{
    std::vector<Armor> result;

    for (std::size_t i = 0; i < lightBars.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lightBars.size(); ++j)
        {
            std::size_t leftIndex = i;
            std::size_t rightIndex = j;
            if (lightBars[rightIndex].center.x < lightBars[leftIndex].center.x)
            {
                std::swap(leftIndex, rightIndex);
            }
            const LightBar& left = lightBars[leftIndex];
            const LightBar& right = lightBars[rightIndex];

            const double minLength = std::min(left.length, right.length);
            if (minLength <= kEpsilon)
            {
                continue;
            }
            const double lengthRatio = std::max(left.length, right.length) / minLength;

            const double angleDiff = std::abs(left.angle - right.angle);
            const double centerYDiff = std::abs(left.center.y - right.center.y);

            const double averageHeight = (left.length + right.length) * 0.5;
            if (averageHeight <= kEpsilon)
            {
                continue;
            }
            const double centerDistanceRatio =
                std::hypot(left.center.x - right.center.x, left.center.y - right.center.y) /
                averageHeight;

            if (lengthRatio > matcherParams.maxLightLengthRatio ||
                angleDiff > matcherParams.maxLightAngleDiffDeg ||
                centerYDiff > matcherParams.maxLightCenterYDiff ||
                centerDistanceRatio < matcherParams.minCenterDistanceRatio ||
                centerDistanceRatio > matcherParams.maxCenterDistanceRatio)
            {
                continue;
            }

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

            const ArmorType type = (centerDistanceRatio >= matcherParams.largeArmorMinCenterDistanceRatio)
                ? ArmorType::Large
                : ArmorType::Small;

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

}
