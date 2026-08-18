#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim
{

// ========== 数据模型 ==========
// 灯条/装甲板/预处理产物/相机标定等在各阶段间流转的数据结构统一收在这里；
// 各阶段头文件只留自己的调参参数与函数声明。

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

// 单根灯条
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

// 单块装甲板
struct Armor
{
    LightBar leftLight;
    LightBar rightLight;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;
    std::string number;          // 分类得到的数字/标签，空表示未分类
    float confidence = 0.0F;     // 分类 softmax 最大概率
    cv::Mat rvec;                // PnP 解出的旋转向量（相机系，3x1 CV_64F），未解算时为空
    cv::Mat tvec;                // PnP 解出的平移向量（相机系，mm，3x1 CV_64F），未解算时为空
};

// 预处理阶段的单个轮廓候选
struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f centerLine;
    double area = 0.0;
};

// 预处理结果：二值图 + 轮廓候选
struct PreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

// 相机标定：内参矩阵、畸变系数；error 为空表示加载成功
struct CameraCalibration
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::string error;
};

} // namespace auto_aim
