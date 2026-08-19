#pragma once

#include <cstdint>

#include "tracker_types.hpp"

namespace auto_aim
{

// 整车状态估计器
class TargetEstimator
{
public:
    // 新一帧进来：用两帧时间戳之差算好本帧 dt_，供 predict / update 用；同时记下时间戳
    void newFrame(std::uint64_t timestamp);

    // 首帧起模型：把整车状态全部重置为默认(速度清零、半径回默认值)，再由观测装甲反推中心
    void init(const ArmorMeasurement& z);

    // 匀速模型预测：位置/高度/角度按 当前值 + 速度 × dt_ 推进；dt_ 由 newFrame 算好
    void predict();

    // 观测更新：把观测装甲反投影成中心/半径观测，算残差后低通修正 state_，半径 clamp 到区间
    void update(const ArmorMeasurement& z);

    // 越过滤波直写 yaw/z(单板特例重锚)：不动中心/速度/半径
    void reanchorYawZ(double yaw, double z);

    const TargetState& state() const { return state_; }

    // 观测函数 h(x)：由当前状态推出装甲板应在的世界位姿，给 Tracker 做匹配/门用
    ArmorMeasurement predictedArmor() const;

    //观测更新的低通增益：posGain 越大越信观测(越灵敏越抖)，velGain 决定残差折进速度的比例
    double posGain = 0.5;        // 位置/角度一阶低通增益 (alpha)
    double velGain = 0.1;        // 速度低通增益 (beta)
    double radiusGain = 0.05;    // 半径低通增益：半径变化慢，取比 posGain 小，避免与中心估计耦合抖动

    //半径估计范围：观测更新时把半径 clamp 到此区间
    double minRadius = 100.0;    // 半径下限, mm
    double maxRadius = 400.0;    // 半径上限, mm

    double tickToSecond = 1.0;   // 设备 tick → 秒 换算系数

private:
    TargetState state_;
    double dt_ = 0.0;                    // 本帧 dt(秒)，由 newFrame 算好，predict/update 用
    std::uint64_t lastTimestamp_ = 0;    // 上一帧硬件时间戳(tick)，用于算 dt_
};

} // namespace auto_aim
