#pragma once

#include <opencv2/core.hpp>

namespace auto_aim
{

// 相机系→世界系外参：机械相机位置 t（相机原点在世界系下的平移，3x1 CV_64F，单位 mm）
// 目前是占位零向量，上车按机械实测填；R 每帧由 IMU 四元数给出，不放这里
struct CameraToWorldParams
{
    cv::Mat cameraTranslation = cv::Mat::zeros(3, 1, CV_64F);
};

// 追踪器：接 detector 的相机系位姿，第一步用 IMU 四元数 + 机械外参把坐标转到世界系
class Tracker
{
public:
    explicit Tracker(const CameraToWorldParams& params = {});

    // 把相机系下的点(3x1 CV_64F, mm)转换到世界系：IMU 四元数构 R + 机械外参 t 拼 4x4，齐次矩阵乘
    cv::Mat cameraToWorld(const cv::Mat& pointInCamera, const cv::Vec4d& quaternion) const;

private:
    CameraToWorldParams params_;
};

} // namespace auto_aim
