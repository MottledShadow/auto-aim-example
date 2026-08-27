#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"

namespace auto_aim::detector
{

// ========== 调参结构体 ==========

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
    LightColor targetColor = LightColor::Red;  // 目标灯条颜色，非此色的灯条舍弃
};

struct LightBarMatcherParams
{
    double maxLightLengthRatio = 1.5;                  // 两灯条长度比上限（长/短）
    double maxLightAngleDiffDeg = 10.0;                // 两灯条角度差上限（度）// TODO 上车重点调整
    double maxLightCenterYDiff = 30.0;                 // 两灯条中心 y 差上限（像素）
    double minCenterDistanceRatio = 2.0;               // 中心距/平均灯条长 下限
    double maxCenterDistanceRatio = 6.0;               // 中心距/平均灯条长 上限
    double largeArmorMinCenterDistanceRatio = 4.0;     // 中心距比 ≥ 此值判大装甲
};

// 无状态的几何检测器：只做预处理/灯条筛选/装甲配对三个几何阶段，不含数字分类与 PnP。
// 参数以公开成员暴露，三个阶段各一方法。
class LightbarDetector
{
public:
    int binaryThreshold = 100;  // 灰度二值化阈值
    LightBarFilterParams filterParams;
    LightBarMatcherParams matcherParams;

    PreprocessResult preprocess(const cv::Mat& frame) const;

    std::vector<LightBar> filterLightBars(
        const cv::Mat& frame,
        const PreprocessResult& pre) const;

    std::vector<Armor> matchArmors(const std::vector<LightBar>& lightBars) const;
};

} // namespace auto_aim::detector
