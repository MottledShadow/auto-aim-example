#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "detector_types.hpp"

namespace auto_aim
{

// ========== 数据模型 ==========

// 进入追踪器的精简装甲板
struct TrackedArmor
{
    ArmorType type = ArmorType::Unknown;
    std::string number;
    cv::Vec3d position;      // 世界系 FLU 坐标, mm
    cv::Vec4d orientation;   // 世界系装甲板朝向四元数 (w, x, y, z)
};

struct TargetState
{
    double xc = 0.0;     // 机器人中心 x, mm
    double yc = 0.0;     // 机器人中心 y, mm
    double vxc = 0.0;    // 中心 x 速度, mm/s
    double vyc = 0.0;    // 中心 y 速度, mm/s
    double z = 0.0;      // 装甲板高度, mm
    double vz = 0.0;     // 高度速度, mm/s
    double yaw = 0.0;    // 装甲板角度, rad
    double vYaw = 0.0;   // 角速度, rad/s
    double r = 200.0;    // 机器人半径(装甲板到中心距离), mm；默认 0.2m
};

// 估计器的观测量：一块装甲板的世界系位姿，即观测函数 h(x) 的值域
struct ArmorMeasurement
{
    double xa = 0.0;    // 装甲板世界系 x, mm
    double ya = 0.0;    // 装甲板世界系 y, mm
    double za = 0.0;    // 装甲板世界系 z(高度), mm
    double yaw = 0.0;   // 装甲板角度, rad
};

// 追踪状态机
enum class TrackerState { Lost, Detecting, Tracking, TempLost };

} // namespace auto_aim
