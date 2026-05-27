#include "pnp_solver.hpp"

#include <stdexcept>
#include <utility>

#include <opencv2/core/persistence.hpp>

namespace auto_aim
{
namespace
{

constexpr double kMinArmorSize = 1e-6;

void validateParams(const PnpSolverParams& params)
{
    if (params.small_armor_width <= kMinArmorSize ||
        params.small_armor_height <= kMinArmorSize ||
        params.large_armor_width <= kMinArmorSize ||
        params.large_armor_height <= kMinArmorSize)
    {
        throw std::invalid_argument("armor dimensions must be positive");
    }
}

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

void loadCalibrationFile(
    const std::string& path,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open calibration file: " + path);
    }

    storage["camera_matrix"] >> camera_matrix;
    storage["dist_coeffs"] >> dist_coeffs;
    if (camera_matrix.empty() || dist_coeffs.empty())
    {
        throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
    }
}

std::vector<cv::Point2f> makeImagePoints(const Armor& armor)
{
    // This order must match makeObjectPoints().
    return {
        armor.left_light.top,
        armor.right_light.top,
        armor.right_light.bottom,
        armor.left_light.bottom,
    };
}

std::vector<cv::Point3f> makeObjectPoints(const Armor& armor, const PnpSolverParams& params)
{
    const bool is_large = armor.type == ArmorType::Large;
    const float width = static_cast<float>(is_large ? params.large_armor_width : params.small_armor_width);
    const float height = static_cast<float>(is_large ? params.large_armor_height : params.small_armor_height);
    const float half_width = width * 0.5F;
    const float half_height = height * 0.5F;

    // Object coordinates are centered on the armor plate.
    return {
        {-half_width, -half_height, 0.0F},
        { half_width, -half_height, 0.0F},
        { half_width,  half_height, 0.0F},
        {-half_width,  half_height, 0.0F},
    };
}

} // namespace

PnpSolver::PnpSolver(PnpSolverParams params) : params_(std::move(params))
{
    validateParams(params_);

    try
    {
        if (!params_.camera_matrix.empty())
        {
            camera_matrix_ = normalizeCameraMatrix(params_.camera_matrix);
        }
        if (!params_.dist_coeffs.empty())
        {
            dist_coeffs_ = normalizeDistCoeffs(params_.dist_coeffs);
        }

        if (camera_matrix_.empty() && !params_.calibration_file.empty())
        {
            loadCalibrationFile(params_.calibration_file, camera_matrix_, dist_coeffs_);
            camera_matrix_ = normalizeCameraMatrix(camera_matrix_);
            dist_coeffs_ = normalizeDistCoeffs(dist_coeffs_);
        }
    }
    catch (const std::exception& ex)
    {
        camera_matrix_.release();
        dist_coeffs_.release();
        calibration_error_ = ex.what();
    }
}

PnpSolveResult PnpSolver::solve(const ArmorMatchResult& armors) const
{
    PnpSolveResult result;
    result.calibration_ready = calibrationReady();
    result.calibration_error = calibration_error_;
    if (!result.calibration_ready)
    {
        return result;
    }

    for (const auto& candidate : armors.candidates)
    {
        const std::vector<cv::Point2f> image_points = makeImagePoints(candidate.armor);
        const std::vector<cv::Point3f> object_points = makeObjectPoints(candidate.armor, params_);

        cv::Mat rvec;
        cv::Mat tvec;
        const bool solved = cv::solvePnP(
            object_points,
            image_points,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            false,
            params_.solve_pnp_method);
        if (!solved)
        {
            continue;
        }

        ArmorPose pose;
        pose.armor = candidate.armor;
        pose.rvec = rvec;
        pose.tvec = tvec;
        pose.distance = cv::norm(tvec);
        pose.image_points = image_points;
        pose.object_points = object_points;
        result.poses.emplace_back(std::move(pose));
    }

    return result;
}

bool PnpSolver::calibrationReady() const
{
    return !camera_matrix_.empty();
}

const std::string& PnpSolver::calibrationError() const
{
    return calibration_error_;
}

const PnpSolverParams& PnpSolver::params() const
{
    return params_;
}

} // namespace auto_aim
