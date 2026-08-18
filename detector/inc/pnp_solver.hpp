#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "detector_types.hpp"

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

CameraCalibration loadCalibration(const std::string& path = "config/camera_calibration.yml");

// PnP 位姿解算器：构造时持有标定与参数，之后每帧把 rvec/tvec 写回装甲板
class PnpSolver
{
public:
    explicit PnpSolver(const CameraCalibration& calibration, const PnpSolverParams& params = {});

    // 逐块解算并把位姿写进装甲板，返回带位姿的装甲板；解算失败的装甲板被丢弃
    std::vector<Armor> solve(const std::vector<Armor>& armors) const;

private:
    CameraCalibration calibration_;
    PnpSolverParams params_;
};

} // namespace auto_aim
