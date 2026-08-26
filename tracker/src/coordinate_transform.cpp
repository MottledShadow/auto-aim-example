#include "coordinate_transform.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

namespace auto_aim::tracker
{

namespace
{
// 手眼加载失败时的回退：相机光学系(RDF: x右/y下/z前) → 机体系(FLU: x前/y左/z上) 轴向重映射
// cam_z前→body_x前、cam_x右→body_-y、cam_y下→body_-z
cv::Mat fallbackCamToGimbal()
{
    return (cv::Mat_<double>(3, 3) <<
             0,  0,  1,
            -1,  0,  0,
             0, -1,  0);
}
}  // namespace

//读手眼标定：cam→gimbal 旋转(3x3) + 平移(3x1, mm)，路径写死，读不到就抛
static void loadHandEye(cv::Mat& rCamToGimbal, cv::Mat& tCamToGimbal)
{
    const std::string path = "config/hand_eye_calibration.yml";

    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open hand-eye file: " + path);
    }

    storage["cam_to_gimbal_rotation"] >> rCamToGimbal;
    storage["cam_to_gimbal_translation"] >> tCamToGimbal;
    if (rCamToGimbal.empty() || tCamToGimbal.empty())
    {
        throw std::runtime_error("hand-eye file must contain cam_to_gimbal_rotation and cam_to_gimbal_translation");
    }

    //统一成 CV_64F，尺寸不对报错
    rCamToGimbal.convertTo(rCamToGimbal, CV_64F);
    tCamToGimbal.convertTo(tCamToGimbal, CV_64F);
    tCamToGimbal = tCamToGimbal.reshape(1, 3);
    if (rCamToGimbal.rows != 3 || rCamToGimbal.cols != 3 || tCamToGimbal.rows != 3 || tCamToGimbal.cols != 1)
    {
        throw std::runtime_error("cam_to_gimbal_rotation must be 3x3 and cam_to_gimbal_translation 3x1");
    }
}

CoordinateTransform::CoordinateTransform()
{
    try
    {
        //构造时自读手眼标定，填相机→云台机体外参
        loadHandEye(rCamToGimbal_, tCamToGimbal_);
    }
    catch (const std::exception& ex)
    {
        //加载失败：回退到硬编码光学系重映射 + 零平移，把原因记进 error_（软失败，仍可跑）
        rCamToGimbal_ = fallbackCamToGimbal();
        tCamToGimbal_ = cv::Mat::zeros(3, 1, CV_64F);
        error_ = ex.what();
    }
}

std::vector<TrackedArmor> CoordinateTransform::toWorld(const std::vector<detector::Armor>& armors,
                                                       const cv::Vec4d& quaternion) const
{
    //IMU 四元数(w,x,y,z) 直接构 Eigen 四元数，转成机体系→世界系旋转矩阵 R_imu
    Eigen::Quaterniond qImu(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    Eigen::Matrix3d rImu = qImu.toRotationMatrix();

    //手眼：相机→云台机体的旋转与平移
    Eigen::Matrix3d rCam2Gimbal;
    cv::cv2eigen(rCamToGimbal_, rCam2Gimbal);
    Eigen::Vector3d tCam2Gimbal;
    cv::cv2eigen(tCamToGimbal_, tCam2Gimbal);

    std::vector<TrackedArmor> tracked;
    for (const detector::Armor& armor : armors)
    {
        TrackedArmor out;
        out.type = armor.type;
        out.number = armor.number;

        //位置：相机系 tvec 先经手眼搬到云台机体系，再由 IMU 旋到世界系
        Eigen::Vector3d tvec;
        cv::cv2eigen(armor.tvec, tvec);
        Eigen::Vector3d pGimbal = rCam2Gimbal * tvec + tCam2Gimbal;
        Eigen::Vector3d pWorld = rImu * pGimbal;
        out.position = cv::Vec3d(pWorld.x(), pWorld.y(), pWorld.z());

        //朝向：rvec 经 Rodrigues 得装甲系→相机系旋转，左乘手眼与 IMU 到世界系后由 Eigen 转四元数
        cv::Mat rArmorCv;
        cv::Rodrigues(armor.rvec, rArmorCv);
        Eigen::Matrix3d rArmor2Cam;
        cv::cv2eigen(rArmorCv, rArmor2Cam);
        Eigen::Quaterniond q(rImu * rCam2Gimbal * rArmor2Cam);
        out.orientation = cv::Vec4d(q.w(), q.x(), q.y(), q.z());

        tracked.push_back(out);
    }
    return tracked;
}

double CoordinateTransform::orientationToYaw(const cv::Vec4d& q)
{
    //q = (w, x, y, z)，取绕世界系 z 轴的偏航角
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

} // namespace auto_aim::tracker
