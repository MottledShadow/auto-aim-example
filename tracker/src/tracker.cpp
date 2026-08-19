#include "tracker.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace auto_aim
{

Tracker::Tracker(const CameraToWorldParams& params)
    : params_(params)
{
}

std::vector<TrackedArmor> Tracker::track(const DetectionResult& detection)
{
    //本帧输入存进对象，后续各步直接读成员，不再层层传参
    armors_ = detection.armors;
    quaternion_ = detection.quaternion;
    timestamp_ = detection.timestamp;

    toWorld();   //填 tracked_

    //丢失态：没有模型，等一个有效观测重新起模型后进入确认中
    if (trackerState_ == TrackerState::Lost)
    {
        if (!tracked_.empty())
        {
            initStateFromArmor();
            lastTimestamp_ = timestamp_;
            trackerState_ = TrackerState::Detecting;
            detectCount_ = 0;
        }
        return tracked_;
    }

    //已有模型：先按两帧间隔匀速预测，再用本帧观测做位置差/角度差双阈值匹配判定
    predict();
    bool matched = false;
    if (!tracked_.empty())
    {
        matched = update();   //匹配成功才低通修正 state_ 并返回 true，否则靠 predict 滑行
    }

    //按匹配结果推进状态机
    switch (trackerState_)
    {
    case TrackerState::Detecting:
        if (matched)
        {
            //确认中累计匹配帧，够 trackingThreshold 就转追踪
            detectCount_++;
            if (detectCount_ >= trackingThreshold)
                trackerState_ = TrackerState::Tracking;
        }
        else
        {
            //确认中漏一帧就退回丢失、重新起模型
            detectCount_ = 0;
            trackerState_ = TrackerState::Lost;
        }
        break;
    case TrackerState::Tracking:
        //追踪中丢观测：先进暂时丢失，靠 predict 滑行
        if (!matched)
        {
            trackerState_ = TrackerState::TempLost;
            lostCount_ = 1;
        }
        break;
    case TrackerState::TempLost:
        if (matched)
        {
            //暂时丢失期间重新匹配上：回到追踪
            trackerState_ = TrackerState::Tracking;
        }
        else
        {
            //连续滑行超过 lostThreshold 帧：判彻底丢失，下个观测重新 init
            lostCount_++;
            if (lostCount_ > lostThreshold)
                trackerState_ = TrackerState::Lost;
        }
        break;
    default:
        break;
    }

    lastTimestamp_ = timestamp_;

    return tracked_;
}

void Tracker::toWorld()
{
    //IMU 四元数(w,x,y,z) 直接构 Eigen 四元数，转成机体系→世界系旋转矩阵 R_imu
    //若电控发 (x,y,z,w) 改这里的下标顺序
    Eigen::Quaterniond qImu(quaternion_[0], quaternion_[1], quaternion_[2], quaternion_[3]);
    Eigen::Matrix3d rImu = qImu.toRotationMatrix();

    //固定的相机光学系(RDF: x右/y下/z前) → 机体系(FLU: x前/y左/z上) 轴向重映射
    //cam_z前→body_x前、cam_x右→body_-y、cam_y下→body_-z；机械 mount 旋转以后可左乘进来
    Eigen::Matrix3d rCam2Body;
    rCam2Body <<  0,  0,  1,
                 -1,  0,  0,
                  0, -1,  0;

    //相机系→世界系整体旋转：先光学系→机体系，再机体系→世界系
    Eigen::Matrix3d rCam2World = rImu * rCam2Body;

    //相机机械平移 cv::Mat(3x1) → Eigen 向量
    Eigen::Vector3d camT;
    cv::cv2eigen(params_.cameraTranslation, camT);

    tracked_.clear();
    for (const Armor& armor : armors_)
    {
        TrackedArmor out;
        out.type = armor.type;
        out.number = armor.number;

        //位置：相机系 tvec 旋到世界系再加相机机械平移
        Eigen::Vector3d tvec;
        cv::cv2eigen(armor.tvec, tvec);
        Eigen::Vector3d pWorld = rCam2World * tvec + camT;
        out.position = cv::Vec3d(pWorld.x(), pWorld.y(), pWorld.z());

        //朝向：rvec 经 Rodrigues 得装甲系→相机系旋转，左乘到世界系后由 Eigen 直接转四元数
        cv::Mat rArmorCv;
        cv::Rodrigues(armor.rvec, rArmorCv);
        Eigen::Matrix3d rArmor2Cam;
        cv::cv2eigen(rArmorCv, rArmor2Cam);
        Eigen::Quaterniond q(rCam2World * rArmor2Cam);
        out.orientation = cv::Vec4d(q.w(), q.x(), q.y(), q.z());

        tracked_.push_back(out);
    }
}

void Tracker::predict()
{
    //dt = 两帧硬件时间戳之差(设备 tick) × tickToSecond；tickToSecond 未标定时默认 1.0，
    //定标设备 tick 频率后只改该系数即得真实秒。当前速度均为 0、predict 不产生位移
    const double dt = static_cast<double>(timestamp_ - lastTimestamp_) * tickToSecond;
    state_.xc  += state_.vxc  * dt;
    state_.yc  += state_.vyc  * dt;
    state_.z   += state_.vz   * dt;
    state_.yaw += state_.vYaw * dt;
}

bool Tracker::update()
{
    //由预测状态反推预测装甲世界位置（initStateFromArmor 里 xc = xa + r*cos(yaw) 的逆）
    const double xaPred = state_.xc - state_.r * std::cos(state_.yaw);
    const double yaPred = state_.yc - state_.r * std::sin(state_.yaw);
    const double zaPred = state_.z;

    //选距离预测位置最近的观测装甲板（世界系欧氏距离平方）
    std::size_t best = 0;
    double bestDist = 0.0;
    for (std::size_t i = 0; i < tracked_.size(); ++i)
    {
        const cv::Vec3d& p = tracked_[i].position;
        const double dx = p[0] - xaPred;
        const double dy = p[1] - yaPred;
        const double dz = p[2] - zaPred;
        const double d = dx * dx + dy * dy + dz * dz;
        if (i == 0 || d < bestDist) { bestDist = d; best = i; }
    }

    //本帧观测的整车状态：由选中的世界系装甲板反推——中心 xy、高度 z、角度 yaw
    const TrackedArmor& armor = tracked_[best];
    const double yawObs = orientationToYaw(armor.orientation);

    //位置差(mm)与 yaw 角度差(rad)双阈值判匹配：任一超阈就算本帧没匹配上，不修正 state_，靠 predict 滑行
    const double posErr = std::sqrt(bestDist);   //bestDist 是选板时的距离平方，开方即位置差
    const double yawErr = std::abs(std::remainder(yawObs - state_.yaw, 2.0 * M_PI));
    if (posErr > maxMatchDistance || yawErr > maxMatchYaw)
        return false;

    const double zObs = armor.position[2];
    const double xcObs = armor.position[0] + state_.r * std::cos(yawObs);
    const double ycObs = armor.position[1] + state_.r * std::sin(yawObs);

    //dt 与 predict 同一口径（本帧-上帧），把位置残差折算成速度用
    const double dt = static_cast<double>(timestamp_ - lastTimestamp_) * tickToSecond;

    //残差 = 观测 - 预测；yaw 残差绕到 [-pi, pi]，避免过 ±pi 时跳变
    const double rx = xcObs - state_.xc;
    const double ry = ycObs - state_.yc;
    const double rz = zObs - state_.z;
    const double rYaw = std::remainder(yawObs - state_.yaw, 2.0 * M_PI);

    //位置/角度按 posGain 吸收残差（一阶低通）；速度按 velGain 吸收 残差/dt（等价对速度做低通）
    state_.xc  += posGain * rx;
    state_.yc  += posGain * ry;
    state_.z   += posGain * rz;
    state_.yaw += posGain * rYaw;
    state_.vxc  += velGain * rx / dt;
    state_.vyc  += velGain * ry / dt;
    state_.vz   += velGain * rz / dt;
    state_.vYaw += velGain * rYaw / dt;

    return true;
}

double Tracker::orientationToYaw(const cv::Vec4d& q)
{
    //q = (w, x, y, z)，取绕世界系 z 轴的偏航角
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void Tracker::initStateFromArmor()
{
    //选距离画面中心最近的装甲板：比 armors_[i] 像素 center 与 imageCenter 的距离（与 tracked_ 同序对齐）
    std::size_t best = 0;
    double bestDist = 0.0;
    for (std::size_t i = 0; i < armors_.size(); ++i)
    {
        const double dx = armors_[i].center.x - imageCenter.x;
        const double dy = armors_[i].center.y - imageCenter.y;
        const double d = dx * dx + dy * dy;
        if (i == 0 || d < bestDist) { bestDist = d; best = i; }
    }
    const TrackedArmor& armor = tracked_[best];
    //装甲板角度由世界系朝向四元数得到
    const double yaw = orientationToYaw(armor.orientation);
    //装甲板高度直接取世界系坐标 z
    const double z = armor.position[2];
    //半径默认 200mm，取装甲板 xy
    const double r = state_.r;
    const double xa = armor.position[0];
    const double ya = armor.position[1];

    //机器人中心 = 装甲板 xy 沿 yaw 方向偏移一个半径
    //约定：xc = xa + r*cos(yaw), yc = ya + r*sin(yaw)（rm_vision 惯例，符号上车再核）
    state_.xc = xa + r * std::cos(yaw);
    state_.yc = ya + r * std::sin(yaw);
    state_.z = z;
    state_.yaw = yaw;
    //速度全部清零（完全重置语义：重新起模型时不带入上一目标的速度），半径保持默认 200mm
    state_.vxc = state_.vyc = state_.vz = state_.vYaw = 0.0;
}

} // namespace auto_aim
