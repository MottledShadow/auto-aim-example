#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace auto_aim::detector
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
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
    std::string number;
    float confidence = 0.0F;
    cv::Mat rvec;
    cv::Mat tvec;
    float distanceToPrincipalPoint = 0.0F;
};

struct FrameInput
{
    cv::Mat image;
    std::uint64_t timestampNs = 0;
    cv::Vec4d quaternion{1, 0, 0, 0};
};

struct DetectionResult
{
    std::vector<Armor> armors;
    std::uint64_t timestampNs = 0;
    cv::Vec4d quaternion{1, 0, 0, 0};
};

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

struct CameraCalibration
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::string error;
};

}
