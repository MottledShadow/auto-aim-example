#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"
#include "tracker_types.hpp"

namespace auto_aim
{

// 相机系→世界系外参：机械相机位置 t（单位 mm）
struct CameraToWorldParams
{
    cv::Mat cameraTranslation = cv::Mat::zeros(3, 1, CV_64F);
};

// 坐标变换：把 detector 的相机系装甲板转到世界系(FLU)
class CoordinateTransform
{
public:
    explicit CoordinateTransform(const CameraToWorldParams& params = {});

    // 相机系装甲板 + 本帧 IMU 四元数(机体→世界) → 世界系精简装甲板
    // 每帧 IMU 四元数配固定光学系重映射得整体旋转；tvec→世界系位置，rvec(经 Rodrigues)→世界系朝向四元数
    std::vector<TrackedArmor> toWorld(const std::vector<Armor>& armors,
                                      const cv::Vec4d& quaternion) const;

    // 世界系朝向四元数(w,x,y,z) → 绕 z 轴 yaw 角(rad)
    static double orientationToYaw(const cv::Vec4d& quaternion);

private:
    CameraToWorldParams params_;
};

} // namespace auto_aim
