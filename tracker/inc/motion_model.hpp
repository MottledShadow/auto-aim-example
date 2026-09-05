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

    void init(const ArmorMeasurement& z);

    // 用本帧 dt_ 更新 A，计算 x_prior=A*x_post、P_prior=A*P_post*Aᵀ+Q；不覆盖后验。
    void predict();

    // EKF 更新：在 x_prior 处计算 H/K，用观测残差更新 x_post/P_post。
    // 每次观测更新前先 predict；失败抛出异常，不覆盖后验或已保存观测。
    void update(const ArmorMeasurement& z);

    // 越过滤波直写 yaw/z(单板特例重锚)：不动中心/速度/半径
    void reanchorYawZ(double yaw, double z);

    const StateVector& state() const { return x_post; }

    // 观测函数 h(x)：由先验状态推出装甲板应在的世界位姿，给 Tracker 做匹配/门用
    ArmorMeasurement predictedArmor() const;

private:
    StateVector x_post = StateVector::Zero(); // 状态后验
    StateCovarianceMatrix P_post = StateCovarianceMatrix::Identity(); // 协方差后验，init 前为占位值
    StateVector x_prior = StateVector::Zero(); // 状态先验
    StateCovarianceMatrix P_prior = StateCovarianceMatrix::Identity(); // 协方差先验
    MeasurementVector z_ = MeasurementVector::Zero(); // 最近一次实际观测：xa,ya,za,yaw
    TransitionMatrix A;
    MeasurementJacobian H = MeasurementJacobian::Zero();
    NoiseParameters noise; // 构造时填写 Q/R；参数不自动刷新矩阵。
    ErrorParameters error; // 每次 init 时用于重置 P_post。
    ProcessNoiseMatrix Q = ProcessNoiseMatrix::Zero();
    MeasurementNoiseMatrix R = MeasurementNoiseMatrix::Zero();
    void updateA(); // 按当前 dt_ 完整填写 A
    MeasurementVector h(const StateVector& x) const;
    double dt_ = 0.0;                    // 本帧 dt(秒)，由 newFrame 算好，predict/update 用
    bool timestampInitialized_ = false;
    std::uint64_t lastTimestampNs_ = 0;  // 上一帧相机启动后单调纳秒，用于算 dt_
};

} // namespace auto_aim::tracker
