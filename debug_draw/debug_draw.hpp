#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "detector.hpp"

namespace auto_aim
{

//在原图上画候选：轮廓(绿) + 最小外接矩形(黄) + fitLine 中心线(红) + 面积数字
void drawCandidates(cv::Mat& vis, const std::vector<ContourCandidate>& candidates);

//逐候选按 filterLightBars 的公式重算四项指标并标注(绿=在范围/红=超范围)，返回通过候选数
std::size_t drawLightBarMetrics(
    cv::Mat& vis,
    const std::vector<ContourCandidate>& candidates,
    const LightBarFilterParams& params);

//只缩不放：scale=min(w/cols,h/rows)，scale<1 时按 INTER_AREA 等比缩小
void fitToScreen(cv::Mat& image, int screen_w = 1920, int screen_h = 1080);

} // namespace auto_aim
