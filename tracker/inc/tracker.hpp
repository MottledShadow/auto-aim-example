#pragma once

#include <cstdint>
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

// 整车状态模型：机器人中心 xOy + 速度、装甲板高度 + 速度、装甲板角度 + 角速度、机器人半径
// 单位：位置 mm、速度 mm/s、角度 rad、角速度 rad/s；装甲板位置由中心与半径推出
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

// 追踪器：接 detector 的相机系位姿，第一步用 IMU 四元数 + 机械外参把坐标转到世界系
class Tracker
{
public:
    explicit Tracker(const CameraToWorldParams& params = {});

    // 追踪主流程：坐标变换 → 首帧初始化 / 后续帧预测
    // timestamp 为硬件时间戳(设备 tick)，两帧之差经 tickToSecond 换算即预测用的 dt
    std::vector<TrackedArmor> track(const std::vector<Armor>& armors,
                                    const cv::Vec4d& quaternion,
                                    std::uint64_t timestamp);

    bool initialized() const { return initialized_; }
    const TargetState& state() const { return state_; }

    double tickToSecond = 1.0;   // 设备 tick → 秒 换算系数；未标定默认 1.0，标定后只改这里

private:
    // 相机系装甲板 → 世界系精简装甲板：每帧 IMU 四元数(机体→世界) + 固定光学系重映射
    // tvec→世界系位置，rvec(经 Rodrigues)→世界系朝向四元数，携带 type/number。读 armors_/quaternion_，写 tracked_
    void toWorld();

    // 用第一帧世界系装甲板(tracked_.front())初始化整车状态：z 取自坐标、yaw 由四元数、中心由 yaw+xy+r 推出、速度置零
    void initStateFromArmor();

    // 匀速模型预测：位置/高度/角度按 当前值 + 速度 × dt 推进（dt 由成员时间戳算）
    void predict();

    // 世界系朝向四元数(w,x,y,z) → 绕 z 轴 yaw 角(rad)
    static double orientationToYaw(const cv::Vec4d& quaternion);

    CameraToWorldParams params_;
    TargetState state_;
    bool initialized_ = false;

    std::vector<Armor> armors_;          // 本帧相机系装甲板(输入)
    cv::Vec4d quaternion_{1, 0, 0, 0};   // 本帧 IMU 四元数(输入)
    std::uint64_t timestamp_ = 0;        // 本帧硬件时间戳(tick, 输入)
    std::vector<TrackedArmor> tracked_;  // 本帧世界系装甲板(toWorld 输出/返回值)
    std::uint64_t lastTimestamp_ = 0;    // 上一帧硬件时间戳(tick)，用于算 dt
};

} // namespace auto_aim
