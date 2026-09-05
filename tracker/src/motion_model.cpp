#include "motion_model.hpp"

#include <cmath>
#include <stdexcept>

namespace auto_aim::tracker
{

TargetEstimator::TargetEstimator()
{
    state_(Radius) = 200.0;
}

TargetEstimator::TransitionMatrix TargetEstimator::transitionMatrix() const
{
    if (!std::isfinite(dt_) || dt_ < 0.0)
        throw std::invalid_argument("motion model dt must be finite and nonnegative");
    TransitionMatrix a = TransitionMatrix::Identity();
    a(Xc, Vxc) = a(Yc, Vyc) = a(Za, Vza) = a(Yaw, Vyaw) = dt_;
    return a;
}

TargetEstimator::MeasurementVector TargetEstimator::h() const
{
    MeasurementVector predicted;
    predicted << state_(Xc) - state_(Radius) * std::cos(state_(Yaw)),
                 state_(Yc) - state_(Radius) * std::sin(state_(Yaw)), state_(Za), state_(Yaw);
    return predicted;
}

TargetEstimator::MeasurementJacobian TargetEstimator::H() const
{
    MeasurementJacobian jacobian = MeasurementJacobian::Zero();
    const double sine = std::sin(state_(Yaw));
    const double cosine = std::cos(state_(Yaw));
    jacobian(0, Xc) = jacobian(1, Yc) = jacobian(2, Za) = jacobian(3, Yaw) = 1.0;
    jacobian(0, Yaw) = state_(Radius) * sine;
    jacobian(0, Radius) = -cosine;
    jacobian(1, Yaw) = -state_(Radius) * cosine;
    jacobian(1, Radius) = -sine;
    return jacobian;
}

TargetEstimator::ProcessNoiseMatrix TargetEstimator::Q() const
{
    ProcessNoiseMatrix q = ProcessNoiseMatrix::Zero();
    for (int i = 0; i < 9; ++i)
    {
        const double variance = noise.processVariances[i];
        if (!std::isfinite(variance) || variance <= 0.0)
            throw std::invalid_argument("Q variances must be finite and positive");
        q(i, i) = variance;
    }
    for (int i = 0; i < 4; ++i)
    {
        const int position = 2 * i;
        const double covariance = noise.positionVelocityCovariances[i];
        // 用相关系数检查正定性，避免直接平方或方差相乘溢出。
        const double correlation = covariance / std::sqrt(q(position, position))
            / std::sqrt(q(position + 1, position + 1));
        if (!std::isfinite(covariance) || !std::isfinite(correlation)
            || std::abs(correlation) >= 1.0)
            throw std::invalid_argument("Q position/velocity covariance must have |correlation| < 1");
        q(position, position + 1) = q(position + 1, position) = covariance;
    }
    return q;
}

TargetEstimator::MeasurementNoiseMatrix TargetEstimator::R() const
{
    MeasurementNoiseMatrix r = MeasurementNoiseMatrix::Zero();
    for (int i = 0; i < 4; ++i)
    {
        const double variance = noise.measurementVariances[i];
        if (!std::isfinite(variance) || variance <= 0.0)
            throw std::invalid_argument("R variances must be finite and positive");
        r(i, i) = variance;
    }
    return r;
}

void TargetEstimator::newFrame(std::uint64_t timestampNs)
{
    // 首帧只建时间基准；之后纳秒差固定乘 1e-9 得秒，不再感知相机设备 tick。
    if (!timestampInitialized_)
    {
        timestampInitialized_ = true;
        lastTimestampNs_ = timestampNs;
        dt_ = 0.0;
        return;
    }
    if (timestampNs <= lastTimestampNs_)
    {
        throw std::runtime_error("tracker timestampNs must advance monotonically");
    }
    dt_ = static_cast<double>(timestampNs - lastTimestampNs_) * 1e-9;
    lastTimestampNs_ = timestampNs;
}

void TargetEstimator::init(const ArmorMeasurement& z)
{
    //先把整车状态全部重置为默认
    state_.setZero();
    state_(Radius) = 200.0;
    z_ << z.xa, z.ya, z.za, z.yaw;

    //装甲板角度/高度直接取观测；半径用默认值
    const double yaw = z.yaw;
    const double r = state_(Radius);

    //机器人中心 = 装甲板 xy 沿 yaw 方向偏移一个半径
    state_(Xc) = z.xa + r * std::cos(yaw);
    state_(Yc) = z.ya + r * std::sin(yaw);
    state_(Za) = z.za;
    state_(Yaw) = yaw;
}

void TargetEstimator::predict()
{
    //按 newFrame 算好的 dt_ 匀速推进；自己不再算 dt
    state_(Xc)  += state_(Vxc)  * dt_;
    state_(Yc)  += state_(Vyc)  * dt_;
    state_(Za)   += state_(Vza)   * dt_;
    state_(Yaw) += state_(Vyaw) * dt_;
}

ArmorMeasurement TargetEstimator::predictedArmor() const
{
    //h(x)：由中心/半径/yaw 推出装甲板世界位姿
    const MeasurementVector predicted = h();
    ArmorMeasurement pred;
    pred.xa = predicted(0);
    pred.ya = predicted(1);
    pred.za = predicted(2);
    pred.yaw = predicted(3);
    return pred;
}

void TargetEstimator::update(const ArmorMeasurement& z)
{
    z_ << z.xa, z.ya, z.za, z.yaw;
    //把观测装甲位置按当前半径、观测 yaw 反算成中心观测
    const double xcObs = z.xa + state_(Radius) * std::cos(z.yaw);
    const double ycObs = z.ya + state_(Radius) * std::sin(z.yaw);

    //观测半径：当前中心估计指向观测装甲的水平向量沿 yaw 方向投影，单独给半径一个观测量
    //注：径向上中心平移与半径变化本就耦合，单帧无法严格分离，这里靠较小的 radiusGain 缓慢收敛
    const double rObs = (state_(Xc) - z.xa) * std::cos(z.yaw) + (state_(Yc) - z.ya) * std::sin(z.yaw);

    //残差 = 观测 - 预测；yaw 残差绕到 [-pi, pi]，避免过 ±pi 时跳变
    const double rx = xcObs - state_(Xc);
    const double ry = ycObs - state_(Yc);
    const double rz = z.za - state_(Za);
    const double rYaw = std::remainder(z.yaw - state_(Yaw), 2.0 * M_PI);
    const double rRadius = rObs - state_(Radius);

    //位置/角度按 posGain 吸收残差（一阶低通）；速度按 velGain 吸收 残差/dt
    state_(Xc)  += posGain * rx;
    state_(Yc)  += posGain * ry;
    state_(Za)   += posGain * rz;
    state_(Yaw) += posGain * rYaw;
    state_(Vxc)  += velGain * rx / dt_;
    state_(Vyc)  += velGain * ry / dt_;
    state_(Vza)   += velGain * rz / dt_;
    state_(Vyaw) += velGain * rYaw / dt_;

    //半径无速度项，按较小的 radiusGain 缓慢低通，再 clamp 到合理区间防跳变
    state_(Radius) += radiusGain * rRadius;
    if (state_(Radius) < minRadius) state_(Radius) = minRadius;
    if (state_(Radius) > maxRadius) state_(Radius) = maxRadius;
}

void TargetEstimator::reanchorYawZ(double yaw, double z)
{
    //越过滤波直接重锚 yaw/z：只在单板特例下用，中心/速度/半径不动
    state_(Yaw) = yaw;
    state_(Za) = z;
}

} // namespace auto_aim::tracker
