#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"

namespace auto_aim::detector
{

struct LightBarFilterParams
{
    double minArea = 10.0;
    double maxArea = 6500.0;
    double minAspectRatio = 2.5;
    double maxAspectRatio = 15.0;
    double minLineAngleDeg = 0.0;
    double maxLineAngleDeg = 15.0;
    double minFillRatio = 0.4;
    double maxFillRatio = 1.0;
    LightColor targetColor = LightColor::Red;
};

struct LightBarMatcherParams
{
    double maxLightLengthRatio = 1.5;
    double maxLightAngleDiffDeg = 10.0;
    double maxLightCenterYDiff = 30.0;
    double minCenterDistanceRatio = 2.0;
    double maxCenterDistanceRatio = 6.0;
    double largeArmorMinCenterDistanceRatio = 4.0;
};

class LightbarDetector
{
public:
    int binaryThreshold = 100;
    LightBarFilterParams filterParams;
    LightBarMatcherParams matcherParams;

    PreprocessResult preprocess(const cv::Mat& frame) const;

    std::vector<LightBar> filterLightBars(
        const cv::Mat& frame,
        const PreprocessResult& pre) const;

    std::vector<Armor> matchArmors(const std::vector<LightBar>& lightBars) const;
};

}
