#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

// ========== 调参结构体 ==========

struct PreprocessParams
{
    int binaryThreshold = 90;  // 灰度二值化阈值
};

struct LightBarFilterParams
{
    double minArea = 10.0;
    double maxArea = 6000.0;
    double minAspectRatio = 2.0;
    double maxAspectRatio = 15.0;
    double minLineAngleDeg = 0.0;
    double maxLineAngleDeg = 15.0;
    double minFillRatio = 0.5;
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

// ========== 数据结构体 ==========

struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f centerLine;
    double area = 0.0;
};

struct PreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

struct LightBar
{
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float angle = 0.0F;
};

enum class ArmorType
{
    Unknown = 0,
    Small,
    Large,
};

struct Armor
{
    LightBar leftLight;
    LightBar rightLight;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;
    std::string number;          // 分类得到的数字/标签，空表示未分类
    float confidence = 0.0F;     // 分类 softmax 最大概率
};

// 无状态的几何检测器：只做预处理/灯条筛选/装甲配对三个几何阶段，不含数字分类与 PnP。
// 把三个参数结构体收成公开成员，三个阶段各一方法。
class GeometryDetector
{
public:
    PreprocessParams preprocessParams;
    LightBarFilterParams filterParams;
    LightBarMatcherParams matcherParams;

    PreprocessResult preprocess(const cv::Mat& frame) const;

    std::vector<LightBar> filterLightBars(
        const cv::Mat& frame,
        const PreprocessResult& pre) const;

    std::vector<Armor> matchArmors(const std::vector<LightBar>& lightBars) const;
};

} // namespace auto_aim
