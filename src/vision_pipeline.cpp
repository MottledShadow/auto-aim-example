#include "vision_pipeline.hpp"

namespace auto_aim
{

std::vector<cv::Point2f> armorCorners(const LightBar& left, const LightBar& right)
{
    return {
        left.top,
        right.top,
        right.bottom,
        left.bottom,
    };
}

VisionPipelineResult runPipeline(const cv::Mat& frame, const Calibration& calibration)
{
    VisionPipelineResult result;
    result.preprocess = preprocess(frame);
    result.light_bars = filterLightBars(frame, result.preprocess);
    result.armors = matchArmors(result.light_bars);
    result.pnp = solvePnp(result.armors, calibration);
    return result;
}

} // namespace auto_aim
