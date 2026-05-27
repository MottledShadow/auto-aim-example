#pragma once

#include <opencv2/core.hpp>

#include "armor_matcher.hpp"
#include "armor_preprocessor.hpp"
#include "light_bar_filter.hpp"
#include "pnp_solver.hpp"

namespace auto_aim
{

struct VisionPipelineParams
{
    ArmorPreprocessParams preprocess;
    LightBarFilterParams light_filter;
    ArmorMatcherParams armor_matcher;
    PnpSolverParams pnp;
};

struct VisionPipelineResult
{
    ArmorPreprocessResult preprocess;
    LightBarFilterResult light_bars;
    ArmorMatchResult armors;
    PnpSolveResult pnp;
};

class VisionPipeline
{
public:
    explicit VisionPipeline(const VisionPipelineParams& params);

    VisionPipelineResult process(const cv::Mat& frame) const;

private:
    ArmorPreprocessor preprocessor_;
    LightBarFilter light_bar_filter_;
    ArmorMatcher armor_matcher_;
    PnpSolver pnp_solver_;
};

} // namespace auto_aim
