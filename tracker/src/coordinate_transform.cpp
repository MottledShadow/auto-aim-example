#include "coordinate_transform.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace auto_aim
{

CoordinateTransform::CoordinateTransform(const CameraToWorldParams& params)
    : params_(params)
{
}

std::vector<TrackedArmor> CoordinateTransform::toWorld(const std::vector<Armor>& armors,
                                                       const cv::Vec4d& quaternion) const
{
    //IMU 四元数(w,x,y,z) 直接构 Eigen 四元数，转成机体系→世界系旋转矩阵 R_imu
    Eigen::Quaterniond qImu(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    Eigen::Matrix3d rImu = qImu.toRotationMatrix();

    //相机光学系(RDF: x右/y下/z前) → 机体系(FLU: x前/y左/z上) 轴向重映射
    //cam_z前→body_x前、cam_x右→body_-y、cam_y下→body_-z
    Eigen::Matrix3d rCam2Body;
    rCam2Body <<  0,  0,  1,
                 -1,  0,  0,
                  0, -1,  0;

    //相机系→世界系整体旋转
    Eigen::Matrix3d rCam2World = rImu * rCam2Body;

    //相机机械平移
    Eigen::Vector3d camT;
    cv::cv2eigen(params_.cameraTranslation, camT);

    std::vector<TrackedArmor> tracked;
    for (const Armor& armor : armors)
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

} // namespace auto_aim
