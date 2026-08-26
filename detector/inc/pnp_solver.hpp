#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "detector_types.hpp"

namespace auto_aim::detector
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

// PnP 位姿解算器：构造时自己读相机标定，之后每帧把 rvec/tvec 写回装甲板
class PnpSolver
{
public:
    explicit PnpSolver(const PnpSolverParams& params = {});

    // 逐块解算并把位姿写进装甲板，返回带位姿的装甲板；解算失败的装甲板被丢弃
    std::vector<Armor> solve(const std::vector<Armor>& armors) const;

    // 相机标定加载失败原因，空表示成功（失败时 solve 会返回空位姿）
    const std::string& error() const { return calibration_.error; }

private:
    CameraCalibration calibration_;
    PnpSolverParams params_;
};

} // namespace auto_aim::detector
