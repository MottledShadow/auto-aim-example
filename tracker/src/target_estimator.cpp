#include "target_estimator.hpp"

#include <cmath>

namespace auto_aim::tracker
{

void TargetEstimator::newFrame(std::uint64_t timestamp)
{
    //新一帧进来就用两帧时间戳之差算好 dt_，predict / update 直接拿去用
    dt_ = static_cast<double>(timestamp - lastTimestamp_) * tickToSecond;
    lastTimestamp_ = timestamp;
}

void TargetEstimator::init(const ArmorMeasurement& z)
{
    //先把整车状态全部重置为默认
    state_ = TargetState{};

    //装甲板角度/高度直接取观测；半径用默认值
    const double yaw = z.yaw;
    const double r = state_.r;

    //机器人中心 = 装甲板 xy 沿 yaw 方向偏移一个半径
    state_.xc = z.xa + r * std::cos(yaw);
    state_.yc = z.ya + r * std::sin(yaw);
    state_.z = z.za;
    state_.yaw = yaw;
}

void TargetEstimator::predict()
{
    //按 newFrame 算好的 dt_ 匀速推进；自己不再算 dt
    state_.xc  += state_.vxc  * dt_;
    state_.yc  += state_.vyc  * dt_;
    state_.z   += state_.vz   * dt_;
    state_.yaw += state_.vYaw * dt_;
}

ArmorMeasurement TargetEstimator::predictedArmor() const
{
    //h(x)：由中心/半径/yaw 推出装甲板世界位姿
    ArmorMeasurement pred;
    pred.xa = state_.xc - state_.r * std::cos(state_.yaw);
    pred.ya = state_.yc - state_.r * std::sin(state_.yaw);
    pred.za = state_.z;
    pred.yaw = state_.yaw;
    return pred;
}

void TargetEstimator::update(const ArmorMeasurement& z)
{
    //把观测装甲位置按当前半径、观测 yaw 反算成中心观测
    const double xcObs = z.xa + state_.r * std::cos(z.yaw);
    const double ycObs = z.ya + state_.r * std::sin(z.yaw);

    //观测半径：当前中心估计指向观测装甲的水平向量沿 yaw 方向投影，单独给半径一个观测量
    //注：径向上中心平移与半径变化本就耦合，单帧无法严格分离，这里靠较小的 radiusGain 缓慢收敛
    const double rObs = (state_.xc - z.xa) * std::cos(z.yaw) + (state_.yc - z.ya) * std::sin(z.yaw);

    //残差 = 观测 - 预测；yaw 残差绕到 [-pi, pi]，避免过 ±pi 时跳变
    const double rx = xcObs - state_.xc;
    const double ry = ycObs - state_.yc;
    const double rz = z.za - state_.z;
    const double rYaw = std::remainder(z.yaw - state_.yaw, 2.0 * M_PI);
    const double rRadius = rObs - state_.r;

    //位置/角度按 posGain 吸收残差（一阶低通）；速度按 velGain 吸收 残差/dt
    state_.xc  += posGain * rx;
    state_.yc  += posGain * ry;
    state_.z   += posGain * rz;
    state_.yaw += posGain * rYaw;
    state_.vxc  += velGain * rx / dt_;
    state_.vyc  += velGain * ry / dt_;
    state_.vz   += velGain * rz / dt_;
    state_.vYaw += velGain * rYaw / dt_;

    //半径无速度项，按较小的 radiusGain 缓慢低通，再 clamp 到合理区间防跳变
    state_.r += radiusGain * rRadius;
    if (state_.r < minRadius) state_.r = minRadius;
    if (state_.r > maxRadius) state_.r = maxRadius;
}

void TargetEstimator::reanchorYawZ(double yaw, double z)
{
    //越过滤波直接重锚 yaw/z：只在单板特例下用，中心/速度/半径不动
    state_.yaw = yaw;
    state_.z = z;
}

} // namespace auto_aim::tracker
