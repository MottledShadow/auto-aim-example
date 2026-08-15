#include "tracker.hpp"

namespace auto_aim
{

Tracker::Tracker(const CameraToWorldParams& params)
    : params_(params)
{
}

cv::Mat Tracker::cameraToWorld(const cv::Mat& pointInCamera, const cv::Vec4d& quaternion) const
{
    //从 IMU 四元数取四个分量，顺序按 (w, x, y, z)；若电控发 (x, y, z, w) 改这里的下标
    const double w = quaternion[0];
    const double x = quaternion[1];
    const double y = quaternion[2];
    const double z = quaternion[3];

    //用四元数标准公式构建 3x3 旋转矩阵 R（相机系→世界系的姿态）
    cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
        2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y));

    //拼成 4x4 齐次变换 T = [R t; 0 0 0 1]，t 是机械相机位置外参
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));
    params_.cameraTranslation.copyTo(transform(cv::Rect(3, 0, 1, 3)));

    //相机系点补成齐次列向量 [x, y, z, 1]^T
    cv::Mat homogeneous = cv::Mat::ones(4, 1, CV_64F);
    pointInCamera.copyTo(homogeneous.rowRange(0, 3));

    //齐次矩阵乘：p_world = T * p_camera
    cv::Mat worldHomogeneous = transform * homogeneous;

    //去掉齐次分量，返回 3x1 的世界系坐标
    return worldHomogeneous.rowRange(0, 3).clone();
}

} // namespace auto_aim
