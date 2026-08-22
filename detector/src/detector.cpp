#include "detector.hpp"

namespace auto_aim
{

//三件套各自默认参数
Detector::Detector()
    : classifier_(NumberClassifierParams{})
    , pnpSolver_(PnpSolverParams{})
{
}

DetectionResult Detector::detect(const FrameInput& input)
{
    //预处理：灰度→二值→形态学→轮廓候选
    const PreprocessResult pre = lightbarDetector_.preprocess(input.image);

    //筛灯条并定颜色
    const std::vector<LightBar> bars = lightbarDetector_.filterLightBars(input.image, pre);

    //同色灯条配对成装甲
    const std::vector<Armor> armors = lightbarDetector_.matchArmors(bars);

    //数字分类，写回 number/confidence 并按规则去留
    const std::vector<Armor> classified = classifier_.classify(input.image, armors);

    //解位姿
    const std::vector<Armor> solved = pnpSolver_.solve(classified);

    //打包结果，把取帧时刻的时间戳/四元数原样透传给追踪器
    return DetectionResult{solved, input.timestamp, input.quaternion};
}

} // namespace auto_aim
