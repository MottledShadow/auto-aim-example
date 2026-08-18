#include "detector.hpp"

namespace auto_aim
{

//三件套各自默认参数（走头文件），PnP 需要相机标定
Detector::Detector(const CameraCalibration& calibration)
    : classifier_(NumberClassifierParams{})
    , pnpSolver_(calibration, PnpSolverParams{})
{
}

std::vector<Armor> Detector::detect(const cv::Mat& frame)
{
    //预处理：灰度→二值→形态学→轮廓候选
    const PreprocessResult pre = lightbarDetector_.preprocess(frame);

    //筛灯条并定颜色
    const std::vector<LightBar> bars = lightbarDetector_.filterLightBars(frame, pre);

    //同色灯条配对成装甲
    const std::vector<Armor> armors = lightbarDetector_.matchArmors(bars);

    //数字分类，写回 number/confidence 并按规则去留
    const std::vector<Armor> classified = classifier_.classify(frame, armors);

    //解位姿
    return pnpSolver_.solve(classified);
}

} // namespace auto_aim
