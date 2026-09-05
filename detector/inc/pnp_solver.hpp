#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "detector_types.hpp"

namespace auto_aim::detector
{

struct PnpSolverParams
{
    double smallArmorWidth = 130.0;
    double smallArmorHeight = 60.0;
    double largeArmorWidth = 230.0;
    double largeArmorHeight = 55.0;
    int solvePnpMethod = cv::SOLVEPNP_IPPE;
};

class PnpSolver
{
public:
    explicit PnpSolver(const PnpSolverParams& params = {});

    std::vector<Armor> solve(const std::vector<Armor>& armors) const;

    const std::string& error() const { return calibration_.error; }

private:
    CameraCalibration calibration_;
    PnpSolverParams params_;
};

}
