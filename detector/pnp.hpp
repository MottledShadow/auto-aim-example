#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "detector.hpp"

namespace auto_aim
{

// PnP 位姿解算参数：装甲板物理尺寸(mm) + solvePnP 方法（尺寸暂沿用占位值）
struct PnpSolverParams
{
    double small_armor_width = 130.0;
    double small_armor_height = 60.0;
    double large_armor_width = 230.0;
    double large_armor_height = 55.0;
    int solve_pnp_method = cv::SOLVEPNP_IPPE;
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

CameraCalibration loadCalibration(const std::string& path = "config/camera_calibration.yml");

std::vector<ArmorPose> solvePnp(
    const std::vector<Armor>& armors,
    const CameraCalibration& calibration,
    const PnpSolverParams& params = {});

} // namespace auto_aim
