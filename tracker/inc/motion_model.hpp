#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Core>

#include "tracker_types.hpp"

namespace auto_aim::tracker
{

// 整车状态估计器
class TargetEstimator
{
public:
    using StateVector = Eigen::Matrix<double, 9, 1>;
    using TransitionMatrix = Eigen::Matrix<double, 9, 9>;
    using MeasurementVector = Eigen::Matrix<double, 4, 1>;
    using MeasurementJacobian = Eigen::Matrix<double, 4, 9>;
    using ProcessNoiseMatrix = Eigen::Matrix<double, 9, 9>;
    using MeasurementNoiseMatrix = Eigen::Matrix<double, 4, 4>;

    // x 顺序固定；长度 mm，时间 s，角度 rad。
    enum StateIndex { Xc, Vxc, Yc, Vyc, Za, Vza, Yaw, Vyaw, Radius };

    struct NoiseParameters
    {
        // Q 对角线：xc,vxc,yc,vyc,za,vza,yaw,vyaw,r。
        // 单位依次为 mm²,(mm/s)²,mm²,(mm/s)²,mm²,(mm/s)²,rad²,(rad/s)²,mm²。
        std::array<double, 9> processVariances{
            100.0, 10000.0, 100.0, 10000.0, 100.0, 10000.0, 0.001, 0.1, 1.0};
        // Q 对称协方差：xc/vxc,yc/vyc,za/vza,yaw/vyaw；前三项 mm²/s，末项 rad²/s。
        std::array<double, 4> positionVelocityCovariances{100.0, 100.0, 100.0, 0.001};
        // R 对角线：xa,ya,za,yaw；单位 mm²,mm²,mm²,rad²。
        std::array<double, 4> measurementVariances{100.0, 100.0, 100.0, 0.01};
    };

    TargetEstimator();

    // A 的结构固定，四个位置/速度系数使用本帧 dt（秒）。
    TransitionMatrix transitionMatrix() const;
    MeasurementVector h() const;
    // 使用当前 state_ 求 dh/dx；后续 EKF 中在预测后调用。
    MeasurementJacobian H() const;

    // 固定每帧 Q/R，不随 dt 缩放。参数未经实测标定，后续可重新设置。
    // 要求参数有限、方差 > 0、每组 cov² < var_position * var_velocity。
    // 每次构造读取 noise，非法输入抛出 invalid_argument。
    NoiseParameters noise;
    ProcessNoiseMatrix Q() const;
    MeasurementNoiseMatrix R() const;

    // 新一帧进来：用两帧时间戳之差算好本帧 dt_，供 predict / update 用；同时记下时间戳
    void newFrame(std::uint64_t timestampNs);

    // 首帧起模型：把整车状态全部重置为默认(速度清零、半径回默认值)，再由观测装甲反推中心
    void init(const ArmorMeasurement& z);

    // 匀速模型预测：位置/高度/角度按 当前值 + 速度 × dt_ 推进；dt_ 由 newFrame 算好
    void predict();

    // 观测更新：把观测装甲反投影成中心/半径观测，算残差后低通修正 state_，半径 clamp 到区间
    void update(const ArmorMeasurement& z);

    // 越过滤波直写 yaw/z(单板特例重锚)：不动中心/速度/半径
    void reanchorYawZ(double yaw, double z);

    const StateVector& state() const { return state_; }

    // 观测函数 h(x)：由当前状态推出装甲板应在的世界位姿，给 Tracker 做匹配/门用
    ArmorMeasurement predictedArmor() const;

    //观测更新的低通增益：posGain 越大越信观测(越灵敏越抖)，velGain 决定残差折进速度的比例
    double posGain = 0.5;        // 位置/角度一阶低通增益 (alpha)
    double velGain = 0.1;        // 速度低通增益 (beta)
    double radiusGain = 0.05;    // 半径低通增益：半径变化慢，取比 posGain 小，避免与中心估计耦合抖动

    //半径估计范围：观测更新时把半径 clamp 到此区间
    double minRadius = 100.0;    // 半径下限, mm
    double maxRadius = 400.0;    // 半径上限, mm

private:
    StateVector state_ = StateVector::Zero();
    MeasurementVector z_ = MeasurementVector::Zero(); // 最近一次实际观测：xa,ya,za,yaw
    double dt_ = 0.0;                    // 本帧 dt(秒)，由 newFrame 算好，predict/update 用
    bool timestampInitialized_ = false;
    std::uint64_t lastTimestampNs_ = 0;  // 上一帧相机启动后单调纳秒，用于算 dt_
};

} // namespace auto_aim::tracker
