#include "pnp_solver.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

namespace auto_aim::detector
{

static CameraCalibration loadCalibration()
{
    const std::string path = "config/camera_calibration.yml";
    CameraCalibration calibration;

    try
    {
        cv::FileStorage storage(path, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open calibration file: " + path);
        }

        storage["camera_matrix"] >> calibration.cameraMatrix;
        storage["dist_coeffs"] >> calibration.distCoeffs;
        if (calibration.cameraMatrix.empty() || calibration.distCoeffs.empty())
        {
            throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
        }

        calibration.cameraMatrix.convertTo(calibration.cameraMatrix, CV_64F);
        if (calibration.cameraMatrix.rows != 3 || calibration.cameraMatrix.cols != 3)
        {
            throw std::runtime_error("camera_matrix must be 3x3");
        }

        calibration.distCoeffs.convertTo(calibration.distCoeffs, CV_64F);
        calibration.distCoeffs = calibration.distCoeffs.reshape(1, 1).clone();
    }
    catch (const std::exception& ex)
    {
        calibration.cameraMatrix.release();
        calibration.distCoeffs.release();
        calibration.error = ex.what();
    }

    return calibration;
}

PnpSolver::PnpSolver(const PnpSolverParams& params)
    : calibration_(loadCalibration()), params_(params)
{
}

std::vector<Armor> PnpSolver::solve(const std::vector<Armor>& armors) const
{
    std::vector<Armor> solved;

    if (calibration_.cameraMatrix.empty())
    {
        return solved;
    }

    for (const Armor& armor : armors)
    {
        const std::vector<cv::Point2f> imagePoints = {
            armor.leftLight.top,
            armor.rightLight.top,
            armor.rightLight.bottom,
            armor.leftLight.bottom,
        };

        const bool isLarge = armor.type == ArmorType::Large;
        const float halfWidth = static_cast<float>((isLarge ? params_.largeArmorWidth : params_.smallArmorWidth) * 0.5);
        const float halfHeight = static_cast<float>((isLarge ? params_.largeArmorHeight : params_.smallArmorHeight) * 0.5);
        const std::vector<cv::Point3f> objectPoints = {
            {-halfWidth, -halfHeight, 0.0F},
            { halfWidth, -halfHeight, 0.0F},
            { halfWidth,  halfHeight, 0.0F},
            {-halfWidth,  halfHeight, 0.0F},
        };

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

        Armor result = armor;
        result.rvec = rvec;
        result.tvec = tvec;

        const double cx = calibration_.cameraMatrix.at<double>(0, 2);
        const double cy = calibration_.cameraMatrix.at<double>(1, 2);
        result.distanceToPrincipalPoint = static_cast<float>(std::hypot(armor.center.x - cx, armor.center.y - cy));

        solved.push_back(result);
    }

    return solved;
}

}
