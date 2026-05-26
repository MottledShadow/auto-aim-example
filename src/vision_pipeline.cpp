#include "vision_pipeline.hpp"

namespace auto_aim
{

VisionPipeline::VisionPipeline(const VisionPipelineParams& params)
    : preprocessor_(params.preprocess),
      light_bar_filter_(params.light_filter),
      armor_matcher_(params.armor_matcher)
{
}

VisionPipelineResult VisionPipeline::process(const cv::Mat& frame) const
{
    VisionPipelineResult result;
    result.preprocess = preprocessor_.process(frame);
    result.light_bars = light_bar_filter_.filter(frame, result.preprocess);
    result.armors = armor_matcher_.match(result.light_bars);
    return result;
}

} // namespace auto_aim
