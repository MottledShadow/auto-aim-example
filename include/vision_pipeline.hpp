#pragma once

#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace auto_aim
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

enum class ArmorType
{
    Unknown = 0,
    Small,
    Large,
};

class LightBar
{
public:
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float width = 0.0F;
    float angle = 0.0F;
    float area = 0.0F;
};

class Armor
{
public:
    LightBar left_light;
    LightBar right_light;
    cv::Point2f center;
    ArmorType type = ArmorType::Unknown;
};

struct ArmorPreprocessParams
{
    int binary_threshold = 180;
    int open_kernel_size = 3;
    int close_kernel_size = 3;
    int morph_iterations = 1;
};

struct LightBarFilterParams
{
    double min_area = 5.0;
    double max_area = 1000000.0;
    double min_aspect_ratio = 1.2;
    double max_aspect_ratio = 50.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 45.0;
    double min_fill_ratio = 0.25;
    double max_fill_ratio = 1.0;
};

struct ArmorMatcherParams
{
    double max_light_length_ratio = 2.0;
    double max_light_angle_diff_deg = 10.0;
    double max_light_center_y_diff = 40.0;
    double min_center_distance_ratio = 0.5;
    double max_center_distance_ratio = 8.0;
    double large_armor_min_center_distance_ratio = 3.2;
};

struct PnpSolverParams
{
    double small_armor_width = 135.0;
    double small_armor_height = 55.0;
    double large_armor_width = 230.0;
    double large_armor_height = 55.0;

    int solve_pnp_method = cv::SOLVEPNP_IPPE;
};

struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f center_line;
    double area = 0.0;
};

struct ArmorPreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

struct LightBarFilterResult
{
    std::vector<LightBar> candidates;
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
    ArmorPreprocessResult preprocess;
    LightBarFilterResult light_bars;
    ArmorMatchResult armors;
    PnpSolveResult pnp;
};

std::vector<cv::Point2f> armorCorners(const LightBar& left, const LightBar& right);

ArmorPreprocessResult preprocessFrame(const cv::Mat& frame, const ArmorPreprocessParams& params = {});

LightBarFilterResult filterLightBars(
    const cv::Mat& frame,
    const ArmorPreprocessResult& preprocess,
    const LightBarFilterParams& params = {});

ArmorMatchResult matchArmors(const LightBarFilterResult& light_bars, const ArmorMatcherParams& params = {});

Calibration loadCalibration(const std::string& path = "config/camera_calibration.yml");

PnpSolveResult solvePnp(
    const ArmorMatchResult& armors,
    const Calibration& calibration,
    const PnpSolverParams& params = {});

VisionPipelineResult runPipeline(const cv::Mat& frame, const Calibration& calibration);

} // namespace auto_aim
