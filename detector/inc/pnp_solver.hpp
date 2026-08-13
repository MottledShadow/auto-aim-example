#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "geometry_detector.hpp"

namespace auto_aim
{

// PnP 位姿解算参数：装甲板物理尺寸(mm) + solvePnP 方法（尺寸暂沿用占位值）
struct PnpSolverParams
{
    double smallArmorWidth = 130.0;
    double smallArmorHeight = 60.0;
    double largeArmorWidth = 230.0;
    double largeArmorHeight = 55.0;
    int solvePnpMethod = cv::SOLVEPNP_IPPE;
};

// 相机标定：内参矩阵、畸变系数；error 为空表示加载成功
struct CameraCalibration
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
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

// PnP 位姿解算器：构造时持有标定与参数，之后每帧对配对装甲板 solve
class PnpSolver
{
public:
    explicit PnpSolver(const CameraCalibration& calibration, const PnpSolverParams& params = {});

    std::vector<ArmorPose> solve(const std::vector<Armor>& armors) const;

private:
    CameraCalibration calibration_;
    PnpSolverParams params_;
};

} // namespace auto_aim
