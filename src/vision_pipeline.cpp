#include "vision_pipeline.hpp"

namespace auto_aim
{

VisionPipelineResult VisionPipeline::process(const cv::Mat& frame) const
{
    VisionPipelineResult result;
    result.preprocess = preprocessor_.process(frame);
    result.light_bars = light_bar_filter_.filter(frame, result.preprocess);
    result.armors = armor_matcher_.match(result.light_bars);
    result.pnp = pnp_solver_.solve(result.armors);
    return result;
}

} // namespace auto_aim
