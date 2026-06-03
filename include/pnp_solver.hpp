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

    int solve_pnp_method = cv::SOLVEPNP_IPPE;
};

struct ArmorPose
{
    Armor armor;
    cv::Mat rvec;
    cv::Mat tvec;
};

struct PnpSolveResult
{
    std::string calibration_error;
    std::vector<ArmorPose> poses;
};

class PnpSolver
{
public:
    explicit PnpSolver(PnpSolverParams params = {});

    PnpSolveResult solve(const ArmorMatchResult& armors) const;

private:
    PnpSolverParams params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::string calibration_error_;
};

} // namespace auto_aim
