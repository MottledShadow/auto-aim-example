#pragma once

#include <opencv2/core.hpp>

#include "vision_pipeline.hpp"

namespace auto_aim
{

cv::Mat makeDebugPreview(
    const cv::Mat& frame,
    const PreprocessResult& preprocess,
    const std::vector<LightBar>& light_bars,
    const ArmorMatchResult& armors,
    bool show_binary);

} // namespace auto_aim
