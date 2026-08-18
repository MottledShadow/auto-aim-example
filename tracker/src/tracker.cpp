#include "tracker.hpp"

#include <cmath>

#include <opencv2/calib3d.hpp>

namespace auto_aim
{

Tracker::Tracker(const CameraToWorldParams& params)
    : params_(params)
{
}

std::vector<TrackedArmor> Tracker::track(const std::vector<Armor>& armors,
                                         const cv::Vec4d& quaternion,
                                         std::uint64_t timestamp)
{
    //本帧输入存进对象，后续各步直接读成员，不再层层传参
    armors_ = armors;
    quaternion_ = quaternion;
    timestamp_ = timestamp;

    toWorld();   //填 tracked_

    if (!initialized_)
    {
        //第一帧：用观测初始化状态并记下时间基准
        if (!tracked_.empty())
        {
            initStateFromArmor();
            lastTimestamp_ = timestamp_;
            initialized_ = true;
        }
    }
    else
    {
        //后续帧：按两帧间隔做预测，时间基准滚动更新
        predict();
        lastTimestamp_ = timestamp_;
    }

    return tracked_;
}

void Tracker::toWorld()
{
    //从 IMU 四元数取四个分量，顺序按 (w, x, y, z)；若电控发 (x, y, z, w) 改这里的下标
    const double w = quaternion_[0];
    const double x = quaternion_[1];
    const double y = quaternion_[2];
    const double z = quaternion_[3];

    //用四元数标准公式构建 3x3 旋转矩阵 R_imu（机体系→世界系的姿态）
    cv::Mat rImu = (cv::Mat_<double>(3, 3) <<
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
        2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y));

    //固定的相机光学系(RDF: x右/y下/z前) → 机体系(FLU: x前/y左/z上) 轴向重映射
    //cam_z前→body_x前、cam_x右→body_-y、cam_y下→body_-z；机械 mount 旋转以后可左乘进来
    cv::Mat rCam2Body = (cv::Mat_<double>(3, 3) <<
        0,  0,  1,
       -1,  0,  0,
        0, -1,  0);

    //相机系→世界系整体旋转：先光学系→机体系，再机体系→世界系
    cv::Mat rCam2World = rImu * rCam2Body;

    tracked_.clear();
    for (const Armor& armor : armors_)
    {
        TrackedArmor out;
        out.type = armor.type;
        out.number = armor.number;

        //位置：相机系 tvec 旋到世界系再加相机机械平移
        cv::Mat pWorld = rCam2World * armor.tvec + params_.cameraTranslation;
        out.position = cv::Vec3d(pWorld.at<double>(0), pWorld.at<double>(1), pWorld.at<double>(2));

        //朝向：rvec 经 Rodrigues 得装甲系→相机系旋转，再左乘到世界系
        cv::Mat rArmor2Cam;
        cv::Rodrigues(armor.rvec, rArmor2Cam);
        cv::Mat r = rCam2World * rArmor2Cam;

        //标准「旋转矩阵→四元数」，按 trace 分支保证数值稳定，装配成 (w, x, y, z)
        const double trace = r.at<double>(0, 0) + r.at<double>(1, 1) + r.at<double>(2, 2);
        double qw, qx, qy, qz;
        if (trace > 0.0)
        {
            const double s = std::sqrt(trace + 1.0) * 2.0;
            qw = 0.25 * s;
            qx = (r.at<double>(2, 1) - r.at<double>(1, 2)) / s;
            qy = (r.at<double>(0, 2) - r.at<double>(2, 0)) / s;
            qz = (r.at<double>(1, 0) - r.at<double>(0, 1)) / s;
        }
        else if (r.at<double>(0, 0) > r.at<double>(1, 1) && r.at<double>(0, 0) > r.at<double>(2, 2))
        {
            const double s = std::sqrt(1.0 + r.at<double>(0, 0) - r.at<double>(1, 1) - r.at<double>(2, 2)) * 2.0;
            qw = (r.at<double>(2, 1) - r.at<double>(1, 2)) / s;
            qx = 0.25 * s;
            qy = (r.at<double>(0, 1) + r.at<double>(1, 0)) / s;
            qz = (r.at<double>(0, 2) + r.at<double>(2, 0)) / s;
        }
        else if (r.at<double>(1, 1) > r.at<double>(2, 2))
        {
            const double s = std::sqrt(1.0 + r.at<double>(1, 1) - r.at<double>(0, 0) - r.at<double>(2, 2)) * 2.0;
            qw = (r.at<double>(0, 2) - r.at<double>(2, 0)) / s;
            qx = (r.at<double>(0, 1) + r.at<double>(1, 0)) / s;
            qy = 0.25 * s;
            qz = (r.at<double>(1, 2) + r.at<double>(2, 1)) / s;
        }
        else
        {
            const double s = std::sqrt(1.0 + r.at<double>(2, 2) - r.at<double>(0, 0) - r.at<double>(1, 1)) * 2.0;
            qw = (r.at<double>(1, 0) - r.at<double>(0, 1)) / s;
            qx = (r.at<double>(0, 2) + r.at<double>(2, 0)) / s;
            qy = (r.at<double>(1, 2) + r.at<double>(2, 1)) / s;
            qz = 0.25 * s;
        }
        out.orientation = cv::Vec4d(qw, qx, qy, qz);

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
    const TrackedArmor& armor = tracked_.front();
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
    //速度全部保持默认 0，半径保持默认 200mm
}

} // namespace auto_aim
