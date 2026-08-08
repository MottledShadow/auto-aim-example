#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "detector.hpp"

namespace auto_aim
{

enum class ArmorType
{
    Unknown = 0,
    Small,
    Large,
};

struct Armor
{
    LightBar left_light;
    LightBar right_light;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;
};

struct ArmorMatcherParams
{
    double max_light_length_ratio = 3.0;
    double max_light_angle_diff_deg = 20.0;
    double max_light_center_y_diff = 400.0;
    double min_center_distance_ratio = 1.0;
    double max_center_distance_ratio = 5.0;
    double large_armor_min_center_distance_ratio = 3.2;
};

struct PnpSolverParams
{
    double small_armor_width = 130.0;
    double small_armor_height = 60.0;
    double large_armor_width = 230.0;
    double large_armor_height = 55.0;

    int solve_pnp_method = cv::SOLVEPNP_IPPE;
};

struct ArmorMatchResult
{
    std::vector<Armor> candidates;
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

struct Calibration
{
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::string error;
};

struct VisionPipelineResult
{
    PreprocessResult preprocess;
    std::vector<LightBar> light_bars;
    ArmorMatchResult armors;
    PnpSolveResult pnp;
};

std::vector<cv::Point2f> armorCorners(const LightBar& left, const LightBar& right);

ArmorMatchResult matchArmors(const std::vector<LightBar>& light_bars, const ArmorMatcherParams& params = {});

Calibration loadCalibration(const std::string& path = "config/camera_calibration.yml");

PnpSolveResult solvePnp(
    const ArmorMatchResult& armors,
    const Calibration& calibration,
    const PnpSolverParams& params = {});

VisionPipelineResult runPipeline(const cv::Mat& frame, const Calibration& calibration);

} // namespace auto_aim
