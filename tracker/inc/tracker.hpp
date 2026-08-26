#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "detector_types.hpp"
#include "tracker_types.hpp"
#include "coordinate_transform.hpp"
#include "target_estimator.hpp"

namespace auto_aim
{

// 追踪器 facade
class Tracker
{
public:
    explicit Tracker();

    // 追踪主流程：坐标变换 → 首帧初始化 / 后续帧预测+匹配+更新
    // 入参 DetectionResult 带相机系装甲板 + 时间戳(设备 tick) + IMU 四元数；
    // 估计器在 newFrame 里用两帧时间戳之差算 dt(tick→秒 系数见 estimator_.tickToSecond)。
    // 返回本帧世界系装甲板，整车状态走 state()
    std::vector<TrackedArmor> track(const DetectionResult& detection);

    bool initialized() const { return trackerState_ != TrackerState::Lost; }
    const TargetState& state() const { return estimator_.state(); }
    TrackerState trackerState() const { return trackerState_; }

    //状态机阈值
    int trackingThreshold = 20;      // 确认中累计匹配到这么多帧就转追踪
    int lostThreshold = 20;          // 暂时丢失超过这么多帧就判彻底丢失

    //匹配判定阈值
    double maxMatchDistance = 200.0; // 匹配位置差阈值, mm 
    double maxMatchYaw = 0.4;        // 匹配角度差阈值, rad 

private:
    CoordinateTransform transform_;   // 相机系→世界系坐标变换
    TargetEstimator estimator_;       // 整车状态估计

    TrackerState trackerState_ = TrackerState::Lost;   // 上电默认丢失
    int detectCount_ = 0;   // 确认中累计匹配帧数
    int lostCount_ = 0;     // 暂时丢失累计帧数
    std::string trackedNumber_;   // 当前锁定目标的数字
};

} // namespace auto_aim
