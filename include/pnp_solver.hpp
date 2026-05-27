#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "armor_matcher.hpp"

namespace auto_aim
{

struct PnpSolverParams
{
    double small_armor_width = 135.0;
    double small_armor_height = 55.0;
    double large_armor_width = 230.0;
    double large_armor_height = 55.0;

    std::string calibration_file = "config/camera_calibration.yml";
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    int solve_pnp_method = cv::SOLVEPNP_IPPE;
};

struct ArmorPose
{
    Armor armor;
    cv::Mat rvec;
    cv::Mat tvec;
    double distance = 0.0;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
};

struct PnpSolveResult
{
    bool calibration_ready = false;
    std::string calibration_error;
    std::vector<ArmorPose> poses;
};

class PnpSolver
{
public:
    explicit PnpSolver(PnpSolverParams params = {});

    PnpSolveResult solve(const ArmorMatchResult& armors) const;

    bool calibrationReady() const;
    const std::string& calibrationError() const;
    const PnpSolverParams& params() const;

private:
    PnpSolverParams params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::string calibration_error_;
};

} // namespace auto_aim
