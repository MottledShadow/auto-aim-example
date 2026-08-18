#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"

namespace auto_aim
{

// 进入追踪器的精简装甲板：只留追踪必需的类型/数字 + 世界系(FLU)位姿
struct TrackedArmor
{
    ArmorType type = ArmorType::Unknown;
    std::string number;
    cv::Vec3d position;      // 世界系 FLU 坐标, mm
    cv::Vec4d orientation;   // 世界系装甲板朝向四元数 (w, x, y, z)
};

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

    // 相机系装甲板 → 世界系精简装甲板：每帧 IMU 四元数(机体→世界) + 固定光学系重映射
    // tvec→世界系位置，rvec(经 Rodrigues)→世界系朝向四元数，携带 type/number
    std::vector<TrackedArmor> track(const std::vector<Armor>& armors, const cv::Vec4d& quaternion) const;

private:
    CameraToWorldParams params_;
};

} // namespace auto_aim
