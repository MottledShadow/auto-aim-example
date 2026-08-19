#include "tracker.hpp"

#include <cmath>

namespace auto_aim
{

Tracker::Tracker(const CameraToWorldParams& params)
    : transform_(params)
{
}

std::vector<TrackedArmor> Tracker::track(const DetectionResult& detection)
{
    //第一步：把本帧相机系装甲板 + IMU 四元数转到世界系(与 detection.armors 同序不丢板)
    std::vector<TrackedArmor> tracked = transform_.toWorld(detection.armors, detection.quaternion);

    //新一帧进来先算好 dt_(存进估计器)，后面 predict 直接用
    estimator_.newFrame(detection.timestamp);

    //丢失态：没有模型，等一个有效观测重新起模型后进入确认中
    if (trackerState_ == TrackerState::Lost)
    {
        if (!tracked.empty())
        {
            //选距离相机主点最近的装甲板起模型
            std::size_t best = 0;
            double bestDist = 0.0;
            for (std::size_t i = 0; i < detection.armors.size(); ++i)
            {
                const double d = detection.armors[i].distanceToPrincipalPoint;
                if (i == 0 || d < bestDist) { bestDist = d; best = i; }
            }
            const TrackedArmor& armor = tracked[best];
            trackedNumber_ = armor.number;
            estimator_.init({armor.position[0], armor.position[1], armor.position[2],
                             CoordinateTransform::orientationToYaw(armor.orientation)});

            trackerState_ = TrackerState::Detecting;
            detectCount_ = 0;
        }
        return tracked;
    }

    estimator_.predict();

    //再用本帧观测做位置差/角度差双阈值匹配判定
    bool matched = false;
    if (!tracked.empty())
    {
        //由预测状态推出预测装甲世界位姿，作为匹配/门的参照(观测空间比对)
        const ArmorMeasurement pred = estimator_.predictedArmor();

        //只在与锁定数字相同的板里选离预测装甲最近的
        std::size_t best = 0;
        double bestDist = 0.0;
        int sameNumberCount = 0;
        for (std::size_t i = 0; i < tracked.size(); ++i)
        {
            if (tracked[i].number != trackedNumber_)
                continue;

            const cv::Vec3d& p = tracked[i].position;
            const double dx = p[0] - pred.xa;
            const double dy = p[1] - pred.ya;
            const double dz = p[2] - pred.za;
            const double d = dx * dx + dy * dy + dz * dz;
            if (sameNumberCount == 0 || d < bestDist) { bestDist = d; best = i; }
            ++sameNumberCount;
        }

        if (sameNumberCount > 0)
        {
            //本帧观测的装甲世界位姿：中心 xy、高度 z 取自选中的板，yaw 由朝向四元数得到
            const TrackedArmor& armor = tracked[best];
            const ArmorMeasurement obs{armor.position[0], armor.position[1], armor.position[2],
                                       CoordinateTransform::orientationToYaw(armor.orientation)};

            //位置差(mm)与 yaw 角度差(rad)双阈值判匹配：任一超阈就算本帧没匹配上
            const double posErr = std::sqrt(bestDist);   //bestDist 是选板时的距离平方，开方即位置差
            const double yawErr = std::abs(std::remainder(obs.yaw - pred.yaw, 2.0 * M_PI));
            if (posErr > maxMatchDistance || yawErr > maxMatchYaw)
            {
                //门失败且本帧同号板恰好一块 → 越过滤波把 yaw/z 重锚；仍算没匹配上，
                //让状态机照常进/留 TempLost，靠下一帧在重锚位姿附近匹配来确认(多一层保护)
                if (yawErr > maxMatchYaw && sameNumberCount == 1)
                    estimator_.reanchorYawZ(obs.yaw, obs.za);
            }
            else
            {
                //门通过：低通修正 state_
                estimator_.update(obs);
                matched = true;
            }
        }
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

    return tracked;
}

} // namespace auto_aim
