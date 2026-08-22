#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"
#include "lightbar_detector.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

namespace auto_aim
{

// 生产入口 facade：一次构造，持有几何检测 + 数字分类 + PnP 三件套；
// 每帧 detect(frame) → 带位姿的装甲板（Armor 自带 rvec/tvec）
class Detector
{
public:
    explicit Detector();

    // 完整识别流水线：预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP
    // 入参 FrameInput 带图像+时间戳+IMU 四元数；返回 DetectionResult，把时间戳/四元数原样透传给追踪器
    DetectionResult detect(const FrameInput& input);

    // 分类模型加载失败原因，空表示成功（模型缺失时 detect 会一直返回空）
    const std::string& error() const { return classifier_.error(); }

private:
    LightbarDetector lightbarDetector_;
    NumberClassifier classifier_;
    PnpSolver pnpSolver_;
};

} // namespace auto_aim
