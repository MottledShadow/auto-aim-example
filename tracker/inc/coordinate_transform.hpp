#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"
#include "tracker_types.hpp"

namespace auto_aim
{

// 坐标变换：把 detector 的相机系装甲板转到世界系(FLU)
class CoordinateTransform
{
public:
    explicit CoordinateTransform();

    // 相机系装甲板 + 本帧 IMU 四元数(机体→世界) → 世界系精简装甲板
    // 手眼(cam→gimbal)把 tvec 从相机系搬到云台机体系，再由 IMU 四元数旋到世界系；
    // rvec(经 Rodrigues) 同样一路旋到世界系朝向四元数
    std::vector<TrackedArmor> toWorld(const std::vector<Armor>& armors,
                                      const cv::Vec4d& quaternion) const;

    // 世界系朝向四元数(w,x,y,z) → 绕 z 轴 yaw 角(rad)
    static double orientationToYaw(const cv::Vec4d& quaternion);

    // 手眼标定加载失败原因，空表示成功；失败时回退到硬编码光学系重映射+零平移
    const std::string& error() const { return error_; }

private:
    cv::Mat rCamToGimbal_;   // 相机→云台机体旋转 3x3 CV_64F
    cv::Mat tCamToGimbal_;   // 相机→云台机体平移 3x1 CV_64F, mm
    std::string error_;
};

} // namespace auto_aim
