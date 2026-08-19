#include "pnp_solver.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

namespace auto_aim
{

CameraCalibration loadCalibration(const std::string& path)
{
    CameraCalibration calibration;

    //空路径直接返回空标定（error 为空，交给调用方判断 cameraMatrix 是否可用）
    if (path.empty())
    {
        return calibration;
    }

    try
    {
        //打开 OpenCV YAML，读不到就抛异常走下面的 catch
        cv::FileStorage storage(path, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open calibration file: " + path);
        }

        //读内参矩阵与畸变系数，缺任一项都算失败
        storage["camera_matrix"] >> calibration.cameraMatrix;
        storage["dist_coeffs"] >> calibration.distCoeffs;
        if (calibration.cameraMatrix.empty() || calibration.distCoeffs.empty())
        {
            throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
        }

        //内参统一成 3x3 的 CV_64F，形状不对就报错
        calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
        if (calibration.cameraMatrix.rows != 3 || calibration.cameraMatrix.cols != 3)
        {
            throw std::runtime_error("camera_matrix must be 3x3");
        }

        //畸变系数统一成单行的 CV_64F
        calibration.distCoeffs.convertTo(calibration.distCoeffs, CV_64F);
        calibration.distCoeffs = calibration.distCoeffs.reshape(1, 1).clone();
    }
    catch (const std::exception& ex)
    {
        //加载失败：释放半成品矩阵，把原因记进 error
        calibration.cameraMatrix.release();
        calibration.distCoeffs.release();
        calibration.error = ex.what();
    }

    return calibration;
}

PnpSolver::PnpSolver(const CameraCalibration& calibration, const PnpSolverParams& params)
    : calibration_(calibration), params_(params)
{
}

std::vector<Armor> PnpSolver::solve(const std::vector<Armor>& armors) const
{
    std::vector<Armor> solved;

    //没有可用内参（标定失败或未加载）就返回空结果
    if (calibration_.cameraMatrix.empty())
    {
        return solved;
    }

    for (const Armor& armor : armors)
    {
        //2D 图像点：左上→右上→右下→左下（与灯条上下端点契约一致）
        const std::vector<cv::Point2f> imagePoints = {
            armor.leftLight.top,
            armor.rightLight.top,
            armor.rightLight.bottom,
            armor.leftLight.bottom,
        };

        //3D 物体点：按大小装甲取物理尺寸(mm)，同样按左上→右上→右下→左下排列
        const bool isLarge = armor.type == ArmorType::Large;
        const float halfWidth = static_cast<float>((isLarge ? params_.largeArmorWidth : params_.smallArmorWidth) * 0.5);
        const float halfHeight = static_cast<float>((isLarge ? params_.largeArmorHeight : params_.smallArmorHeight) * 0.5);
        const std::vector<cv::Point3f> objectPoints = {
            {-halfWidth, -halfHeight, 0.0F},
            { halfWidth, -halfHeight, 0.0F},
            { halfWidth,  halfHeight, 0.0F},
            {-halfWidth,  halfHeight, 0.0F},
        };

        //解算位姿，失败的装甲板跳过
        cv::Mat rvec;
        cv::Mat tvec;
        const bool ok = cv::solvePnP(
            objectPoints,
            imagePoints,
            calibration_.cameraMatrix,
            calibration_.distCoeffs,
            rvec,
            tvec,
            false,
            params_.solvePnpMethod);
        if (!ok)
        {
            continue;
        }

        //把位姿写回装甲板副本
        Armor result = armor;
        result.rvec = rvec;
        result.tvec = tvec;

        //算装甲中心到主点(cx,cy)的像素距离
        const double cx = calibration_.cameraMatrix.at<double>(0, 2);
        const double cy = calibration_.cameraMatrix.at<double>(1, 2);
        result.distanceToPrincipalPoint = static_cast<float>(std::hypot(armor.center.x - cx, armor.center.y - cy));

        solved.push_back(result);
    }

    return solved;
}

} // namespace auto_aim
