#include "vision_pipeline.hpp"

#include <stdexcept>
#include <utility>

#include <opencv2/core/persistence.hpp>

namespace auto_aim
{
namespace
{

cv::Mat normalizeCameraMatrix(const cv::Mat& matrix)
{
    cv::Mat converted;
    matrix.convertTo(converted, CV_64F);
    if (converted.rows != 3 || converted.cols != 3)
    {
        throw std::runtime_error("camera_matrix must be 3x3");
    }
    return converted;
}

cv::Mat normalizeDistCoeffs(const cv::Mat& coeffs)
{
    cv::Mat converted;
    coeffs.convertTo(converted, CV_64F);
    if (converted.rows != 1 && converted.cols != 1)
    {
        converted = converted.reshape(1, 1).clone();
    }
    return converted;
}

std::vector<cv::Point3f> makeObjectPoints(const Armor& armor, const PnpSolverParams& params)
{
    const bool is_large = armor.type == ArmorType::Large;
    const float width = static_cast<float>(is_large ? params.large_armor_width : params.small_armor_width);
    const float height = static_cast<float>(is_large ? params.large_armor_height : params.small_armor_height);
    const float half_width = width * 0.5F;
    const float half_height = height * 0.5F;

    return {
        {-half_width, -half_height, 0.0F},
        { half_width, -half_height, 0.0F},
        { half_width,  half_height, 0.0F},
        {-half_width,  half_height, 0.0F},
    };
}

} // namespace

Calibration loadCalibration(const std::string& path)
{
    Calibration calibration;
    if (path.empty())
    {
        return calibration;
    }

    try
    {
        cv::FileStorage storage(path, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open calibration file: " + path);
        }

        storage["camera_matrix"] >> calibration.camera_matrix;
        storage["dist_coeffs"] >> calibration.dist_coeffs;
        if (calibration.camera_matrix.empty() || calibration.dist_coeffs.empty())
        {
            throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
        }

        calibration.camera_matrix = normalizeCameraMatrix(calibration.camera_matrix);
        calibration.dist_coeffs = normalizeDistCoeffs(calibration.dist_coeffs);
    }
    catch (const std::exception& ex)
    {
        calibration.camera_matrix.release();
        calibration.dist_coeffs.release();
        calibration.error = ex.what();
    }

    return calibration;
}

PnpSolveResult solvePnp(
    const ArmorMatchResult& armors,
    const Calibration& calibration,
    const PnpSolverParams& params)
{
    PnpSolveResult result;
    result.calibration_error = calibration.error;
    if (calibration.camera_matrix.empty())
    {
        return result;
    }

    for (const Armor& armor : armors.candidates)
    {
        const std::vector<cv::Point2f> image_points = armorCorners(armor.left_light, armor.right_light);
        const std::vector<cv::Point3f> object_points = makeObjectPoints(armor, params);

        cv::Mat rvec;
        cv::Mat tvec;
        const bool solved = cv::solvePnP(
            object_points,
            image_points,
            calibration.camera_matrix,
            calibration.dist_coeffs,
            rvec,
            tvec,
            false,
            params.solve_pnp_method);
        if (!solved)
        {
            continue;
        }

        ArmorPose pose;
        pose.armor = armor;
        pose.rvec = rvec;
        pose.tvec = tvec;
        result.poses.emplace_back(std::move(pose));
    }

    return result;
}

} // namespace auto_aim
