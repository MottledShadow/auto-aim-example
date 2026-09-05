#include "motion_model.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace auto_aim::tracker
{

TargetEstimator::TargetEstimator()
{
    x_post(Radius) = 200.0;
    updateA();
    const auto positiveFinite = [](double variance)
    {
        return std::isfinite(variance) && variance > 0.0;
    };
    if (!positiveFinite(noise.qXc) || !positiveFinite(noise.qVxc)
        || !positiveFinite(noise.qYc) || !positiveFinite(noise.qVyc)
        || !positiveFinite(noise.qZa) || !positiveFinite(noise.qVza)
        || !positiveFinite(noise.qYaw) || !positiveFinite(noise.qVyaw)
        || !positiveFinite(noise.qRadius))
        throw std::invalid_argument("Q variances must be finite and positive");
    if (!positiveFinite(noise.rXa) || !positiveFinite(noise.rYa)
        || !positiveFinite(noise.rZa) || !positiveFinite(noise.rYaw))
        throw std::invalid_argument("R variances must be finite and positive");

    const auto validCovariance = [](double covariance, double positionVariance, double velocityVariance)
    {
        // 用相关系数检查正定性，避免直接平方或方差相乘溢出。
        const double correlation = covariance / std::sqrt(positionVariance) / std::sqrt(velocityVariance);
        return std::isfinite(covariance) && std::isfinite(correlation) && std::abs(correlation) < 1.0;
    };
    if (!validCovariance(noise.qXcVxc, noise.qXc, noise.qVxc)
        || !validCovariance(noise.qYcVyc, noise.qYc, noise.qVyc)
        || !validCovariance(noise.qZaVza, noise.qZa, noise.qVza)
        || !validCovariance(noise.qYawVyaw, noise.qYaw, noise.qVyaw))
        throw std::invalid_argument("Q position/velocity covariance must have |correlation| < 1");

    Q << noise.qXc,    noise.qXcVxc, 0,            0,            0,            0,            0,              0,              0,
         noise.qXcVxc, noise.qVxc,   0,            0,            0,            0,            0,              0,              0,
         0,            0,           noise.qYc,    noise.qYcVyc, 0,            0,            0,              0,              0,
         0,            0,           noise.qYcVyc, noise.qVyc,   0,            0,            0,              0,              0,
         0,            0,           0,            0,            noise.qZa,    noise.qZaVza, 0,              0,              0,
         0,            0,           0,            0,            noise.qZaVza, noise.qVza,   0,              0,              0,
         0,            0,           0,            0,            0,            0,            noise.qYaw,     noise.qYawVyaw, 0,
         0,            0,           0,            0,            0,            0,            noise.qYawVyaw, noise.qVyaw,    0,
         0,            0,           0,            0,            0,            0,            0,              0,              noise.qRadius;

    R << noise.rXa, 0,         0,         0,
         0,         noise.rYa, 0,         0,
         0,         0,         noise.rZa, 0,
         0,         0,         0,         noise.rYaw;
}

TargetEstimator::MeasurementVector TargetEstimator::h(const StateVector& x) const
{
    MeasurementVector predicted;
    predicted << x(Xc) - x(Radius) * std::cos(x(Yaw)),
                 x(Yc) - x(Radius) * std::sin(x(Yaw)), x(Za), x(Yaw);
    return predicted;
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
    // 先校验全部初始方差，避免失败时部分重置已有跟踪状态。
    const auto positiveFinite = [](double variance)
    {
        return std::isfinite(variance) && variance > 0.0;
    };
    if (!positiveFinite(error.pXc) || !positiveFinite(error.pVxc)
        || !positiveFinite(error.pYc) || !positiveFinite(error.pVyc)
        || !positiveFinite(error.pZa) || !positiveFinite(error.pVza)
        || !positiveFinite(error.pYaw) || !positiveFinite(error.pVyaw)
        || !positiveFinite(error.pRadius))
        throw std::invalid_argument("P_post initial variances must be finite and positive");

    //先把整车状态全部重置为默认
    x_post.setZero();
    x_post(Radius) = 200.0;
    z_ << z.xa, z.ya, z.za, z.yaw;

    //装甲板角度/高度直接取观测；半径用默认值
    const double yaw = z.yaw;
    const double r = x_post(Radius);

    //机器人中心 = 装甲板 xy 沿 yaw 方向偏移一个半径
    x_post(Xc) = z.xa + r * std::cos(yaw);
    x_post(Yc) = z.ya + r * std::sin(yaw);
    x_post(Za) = z.za;
    x_post(Yaw) = yaw;

    // 每次初始化都清除历史协方差，直接使用可调的初始对角方差。
    P_post << error.pXc, 0,          0,         0,          0,         0,          0,          0,           0,
         0,         error.pVxc, 0,         0,          0,         0,          0,          0,           0,
         0,         0,          error.pYc, 0,          0,         0,          0,          0,           0,
         0,         0,          0,         error.pVyc, 0,         0,          0,          0,           0,
         0,         0,          0,         0,          error.pZa, 0,          0,          0,           0,
         0,         0,          0,         0,          0,         error.pVza, 0,          0,           0,
         0,         0,          0,         0,          0,         0,          error.pYaw, 0,           0,
         0,         0,          0,         0,          0,         0,          0,          error.pVyaw, 0,
         0,         0,          0,         0,          0,         0,          0,          0,           error.pRadius;

    // 重建模型时同步先验，清除上一次跟踪的预测结果。
    x_prior = x_post;
    P_prior = P_post;
}

void TargetEstimator::updateA()
{
    A << 1, dt_, 0, 0,   0, 0,   0, 0,   0,
         0, 1,   0, 0,   0, 0,   0, 0,   0,
         0, 0,   1, dt_, 0, 0,   0, 0,   0,
         0, 0,   0, 1,   0, 0,   0, 0,   0,
         0, 0,   0, 0,   1, dt_, 0, 0,   0,
         0, 0,   0, 0,   0, 1,   0, 0,   0,
         0, 0,   0, 0,   0, 0,   1, dt_, 0,
         0, 0,   0, 0,   0, 0,   0, 1,   0,
         0, 0,   0, 0,   0, 0,   0, 0,   1;
}

void TargetEstimator::predict()
{
    // 每次使用 newFrame 算好的 dt_ 重新填写运动矩阵。
    updateA();

    x_prior = A * x_post;
    P_prior = A * P_post * A.transpose() + Q;
}

ArmorMeasurement TargetEstimator::predictedArmor() const
{
    //h(x)：由中心/半径/yaw 推出装甲板世界位姿
    const MeasurementVector predicted = h(x_prior);
    ArmorMeasurement pred;
    pred.xa = predicted(0);
    pred.ya = predicted(1);
    pred.za = predicted(2);
    pred.yaw = predicted(3);
    return pred;
}

void TargetEstimator::update(const ArmorMeasurement& z)
{
    MeasurementVector measurement;
    measurement << z.xa, z.ya, z.za, z.yaw;
    if (!measurement.allFinite())
        throw std::invalid_argument("EKF measurement must be finite");
    if (!x_prior.allFinite() || !P_prior.allFinite())
        throw std::runtime_error("EKF prior must be finite");

    // 在先验状态处计算 dh/dx，列顺序与 x 一致。
    const double sine = std::sin(x_prior(Yaw));
    const double cosine = std::cos(x_prior(Yaw));
    const double radius = x_prior(Radius);
    H << 1, 0, 0, 0, 0, 0, radius * sine,    0, -cosine,
         0, 0, 1, 0, 0, 0, -radius * cosine, 0, -sine,
         0, 0, 0, 0, 1, 0, 0,                0, 0,
         0, 0, 0, 0, 0, 0, 1,                0, 0;

    const Eigen::Matrix<double, 4, 4> S = H * P_prior * H.transpose() + R;
    if (!S.allFinite())
        throw std::runtime_error("EKF innovation covariance must be finite");
    const Eigen::LLT<Eigen::Matrix<double, 4, 4>> decomposition(S);
    if (decomposition.info() != Eigen::Success)
        throw std::runtime_error("EKF innovation covariance must be positive definite");
    const Eigen::Matrix<double, 9, 4> crossCovariance = P_prior * H.transpose();
    // 解 S*Kᵀ=(P_prior*Hᵀ)ᵀ，避免显式求逆。
    const Eigen::Matrix<double, 9, 4> K = decomposition.solve(crossCovariance.transpose()).transpose();

    MeasurementVector innovation = measurement - h(x_prior);
    innovation(3) = std::remainder(innovation(3), 2.0 * M_PI);
    const StateVector nextX = x_prior + K * innovation;
    // Joseph 形式，减小浮点误差对协方差对称性和半正定性的影响。
    const StateCovarianceMatrix correction = StateCovarianceMatrix::Identity() - K * H;
    const StateCovarianceMatrix nextP = correction * P_prior * correction.transpose()
        + K * R * K.transpose();
    if (!K.allFinite() || !nextX.allFinite() || !nextP.allFinite())
        throw std::runtime_error("EKF posterior must be finite");

    x_post = nextX;
    P_post = nextP;
    z_ = measurement;
}

void TargetEstimator::reanchorYawZ(double yaw, double z)
{
    //越过滤波直接重锚 yaw/z：只在单板特例下用，中心/速度/半径不动
    x_post(Yaw) = yaw;
    x_post(Za) = z;
}

} // namespace auto_aim::tracker
