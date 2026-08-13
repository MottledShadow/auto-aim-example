#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "geometry_detector.hpp"

namespace auto_aim
{

//原图与二值图左右拼成一张对照图（二值图转 BGR），各自标题，binary 侧标灰度阈值；方便一眼对比二值化效果
cv::Mat sideBySide(const cv::Mat& original, const cv::Mat& binary, int threshold);

//在原图上画候选：轮廓(绿) + 最小外接矩形(黄) + fitLine 中心线(红) + 面积数字
void drawCandidates(cv::Mat& vis, const std::vector<ContourCandidate>& candidates);

//逐候选按 filterLightBars 的公式重算四项指标：每个亮斑用调色板一色画 轮廓+最小外接矩形+中心线(细线、不标序号)，
//四项 A/AR/ang/fill 铺到左上角图例，行首色块与亮斑框同色作对应，各项 ok 绿/超范围红；返回通过候选数
std::size_t drawLightBarMetrics(
    cv::Mat& vis,
    const std::vector<ContourCandidate>& candidates,
    const LightBarFilterParams& params);

//先给每根灯条在中心标索引(白字)，再对所有同色灯条对按 matchArmors 的公式重算配对指标画装甲框(接受粗线/被拒细线，本对专属色)，
//左上角图例每行 "i+j" 起头(色块与框同色)后跟 LR/ang/dY/dist 各项 ok 绿/超范围红；返回被接受的装甲数
std::size_t drawArmorMetrics(
    cv::Mat& vis,
    const std::vector<LightBar>& lightBars,
    const LightBarMatcherParams& params);

//只缩不放：scale=min(w/cols,h/rows)，scale<1 时按 INTER_AREA 等比缩小
void fitToScreen(cv::Mat& image, int screenW = 1920, int screenH = 1080);

} // namespace auto_aim
