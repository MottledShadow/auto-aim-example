#pragma once

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
    using StateCovarianceMatrix = Eigen::Matrix<double, 9, 9>;
    using TransitionMatrix = Eigen::Matrix<double, 9, 9>;
    using MeasurementVector = Eigen::Matrix<double, 4, 1>;
    using MeasurementJacobian = Eigen::Matrix<double, 4, 9>;
    using ProcessNoiseMatrix = Eigen::Matrix<double, 9, 9>;
    using MeasurementNoiseMatrix = Eigen::Matrix<double, 4, 4>;

    // x 顺序固定；长度 mm，时间 s，角度 rad。
    enum StateIndex { Xc, Vxc, Yc, Vyc, Za, Vza, Yaw, Vyaw, Radius };

    struct NoiseParameters
    {
        // Q 对角线：过程噪声方差，固定每帧，不随 dt 缩放。
        double qXc = 100.0;       // mm²
        double qVxc = 10000.0;    // (mm/s)²
        double qYc = 100.0;       // mm²
        double qVyc = 10000.0;    // (mm/s)²
        double qZa = 100.0;       // mm²
        double qVza = 10000.0;    // (mm/s)²
        double qYaw = 0.001;      // rad²
        double qVyaw = 0.1;       // (rad/s)²
        double qRadius = 1.0;     // mm²

        // Q 对称协方差：各位置与对应速度。
        double qXcVxc = 100.0;    // mm²/s
        double qYcVyc = 100.0;    // mm²/s
        double qZaVza = 100.0;    // mm²/s
        double qYawVyaw = 0.001;  // rad²/s

        // R 对角线：测量噪声方差。
        double rXa = 100.0;       // mm²
        double rYa = 100.0;       // mm²
        double rZa = 100.0;       // mm²
        double rYaw = 0.01;       // rad²
    };

    struct ErrorParameters
    {
        // P 初始对角线：估计误差方差，与 Q/R 独立；默认 1 暂作占位。
        double pXc = 1.0;         // mm²
        double pVxc = 1.0;        // (mm/s)²
        double pYc = 1.0;         // mm²
        double pVyc = 1.0;        // (mm/s)²
        double pZa = 1.0;         // mm²
        double pVza = 1.0;        // (mm/s)²
        double pYaw = 1.0;        // rad²
        double pVyaw = 1.0;       // (rad/s)²
        double pRadius = 1.0;     // mm²
    };

    TargetEstimator();

    void newFrame(std::uint64_t timestampNs);

    // 重置速度/半径，由观测装甲反推中心，并按 error.pXXX 重置 P；保留时间基准。
    // P 初始方差须有限且 > 0，否则抛出 invalid_argument，不修改状态、观测和 P。
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
    StateCovarianceMatrix P = StateCovarianceMatrix::Identity(); // init 前的占位值
    MeasurementVector z_ = MeasurementVector::Zero(); // 最近一次实际观测：xa,ya,za,yaw
    TransitionMatrix A;
    MeasurementJacobian H = MeasurementJacobian::Zero();
    NoiseParameters noise; // 构造时填写 Q/R；参数不自动刷新矩阵。
    ErrorParameters error; // 每次 init 时用于重置 P。
    ProcessNoiseMatrix Q = ProcessNoiseMatrix::Zero();
    MeasurementNoiseMatrix R = MeasurementNoiseMatrix::Zero();
    MeasurementVector h(const StateVector& x) const;
    double dt_ = 0.0;                    // 本帧 dt(秒)，由 newFrame 算好，predict/update 用
    bool timestampInitialized_ = false;
    std::uint64_t lastTimestampNs_ = 0;  // 上一帧相机启动后单调纳秒，用于算 dt_
};

} // namespace auto_aim::tracker
