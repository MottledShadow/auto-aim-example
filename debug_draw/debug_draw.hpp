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

//对所有同色灯条对按 matchArmors 的公式重算配对指标并画装甲框(整体接受绿/被拒红)，逐项参数单独着色；返回被接受的装甲数
std::size_t drawArmorMetrics(
    cv::Mat& vis,
    const std::vector<LightBar>& light_bars,
    const LightBarMatcherParams& params);

//只缩不放：scale=min(w/cols,h/rows)，scale<1 时按 INTER_AREA 等比缩小
void fitToScreen(cv::Mat& image, int screen_w = 1920, int screen_h = 1080);

} // namespace auto_aim
