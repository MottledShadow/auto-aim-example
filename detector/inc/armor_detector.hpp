#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "geometry_detector.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

namespace auto_aim
{

// 生产入口 facade：一次构造，持有几何检测 + 数字分类 + PnP 三件套；
// 每帧 detect(frame) → 带位姿的装甲板。追踪器只依赖这一个类和 ArmorPose。
// 调参仍走各 *Params 的头文件默认值，故构造只需相机标定。
class ArmorDetector
{
public:
    explicit ArmorDetector(const CameraCalibration& calibration);

    // 完整识别流水线：预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP
    std::vector<ArmorPose> detect(const cv::Mat& frame);

    // 分类模型加载失败原因，空表示成功（模型缺失时 detect 会一直返回空）
    const std::string& error() const { return classifier_.error(); }

private:
    GeometryDetector detector_;
    NumberClassifier classifier_;
    PnpSolver pnpSolver_;
};

} // namespace auto_aim
