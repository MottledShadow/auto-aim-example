#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace auto_aim
{

enum class LightColor
{
    Unknown = 0,
    Red,
    Blue,
};

// ========== 调参结构体（置顶，方便调参）==========

struct PreprocessParams
{
    int binary_threshold = 90;  // 灰度二值化阈值
};

struct LightBarFilterParams
{
    double min_area = 10.0;
    double max_area = 6000.0;
    double min_aspect_ratio = 2.0;
    double max_aspect_ratio = 15.0;
    double min_line_angle_deg = 0.0;
    double max_line_angle_deg = 15.0;
    double min_fill_ratio = 0.5;
    double max_fill_ratio = 1.0;
    LightColor target_color = LightColor::Red;  // 目标灯条颜色，非此色的灯条舍弃
};

struct LightBarMatcherParams
{
    double max_light_length_ratio = 1.2;                 // 两灯条长度比上限（长/短）
    double max_light_angle_diff_deg = 10.0;              // 两灯条角度差上限（度）// TODO 上车重点调整
    double max_light_center_y_diff = 10.0;              // 两灯条中心 y 差上限（像素）
    double min_center_distance_ratio = 2.3;              // 中心距/平均灯条长 下限
    double max_center_distance_ratio = 6.0;              // 中心距/平均灯条长 上限
    double large_armor_min_center_distance_ratio = 4.0;  // 中心距比 ≥ 此值判大装甲
};

// PnP 位姿解算参数：装甲板物理尺寸(mm) + solvePnP 方法（尺寸暂沿用占位值）
struct PnpSolverParams
{
    double small_armor_width = 130.0;
    double small_armor_height = 60.0;
    double large_armor_width = 230.0;
    double large_armor_height = 55.0;
    int solve_pnp_method = cv::SOLVEPNP_IPPE;
};

// ========== 数据结构体 ==========

struct ContourCandidate
{
    std::vector<cv::Point> contour;
    cv::RotatedRect rect;
    cv::Vec4f center_line;
    double area = 0.0;
};

struct PreprocessResult
{
    cv::Mat binary;
    std::vector<ContourCandidate> candidates;
};

struct LightBar
{
    LightColor color = LightColor::Unknown;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    float length = 0.0F;
    float angle = 0.0F;
};

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

// 相机标定：内参矩阵、畸变系数；error 为空表示加载成功
struct CameraCalibration
{
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::string error;
};

// 单块装甲板的位姿（相机坐标系下的旋转/平移向量）
struct ArmorPose
{
    Armor armor;
    cv::Mat rvec;
    cv::Mat tvec;
};

// ========== 函数接口 ==========

PreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params = {});

std::vector<LightBar> filterLightBars(
    const cv::Mat& frame,
    const PreprocessResult& preprocess,
    const LightBarFilterParams& params = {});

std::vector<Armor> matchArmors(
    const std::vector<LightBar>& light_bars,
    const LightBarMatcherParams& params = {});

CameraCalibration loadCalibration(const std::string& path = "config/camera_calibration.yml");

std::vector<ArmorPose> solvePnp(
    const std::vector<Armor>& armors,
    const CameraCalibration& calibration,
    const PnpSolverParams& params = {});

} // namespace auto_aim
