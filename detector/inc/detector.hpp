#pragma once

#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_types.hpp"
#include "latest_slot.hpp"
#include "lightbar_detector.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

namespace auto_aim
{

// 帧源回调：填好一帧 FrameInput 返回 true；暂时没帧返回 false（线程 continue）
// 回调应自行阻塞等帧（如包 HikCamera::capture 的阻塞超时），避免 false 时空转
using FrameSource = std::function<bool(FrameInput&)>;

// 生产入口 facade：仿 Serial，构造时起后台线程持续跑流水线、发布最新结果，主线程 latest() 取。
// 持有几何检测 + 数字分类 + PnP 三件套；每帧 detect() → 带位姿的装甲板
class Detector
{
public:
    // 构造时注入帧源回调并起后台识别线程
    explicit Detector(FrameSource source);
    ~Detector();

    // 取最近一帧识别结果
    DetectionResult latest();

    // 分类模型加载失败原因，空表示成功
    const std::string& error() const { return classifier_.error(); }

private:
    // 后台线程体：帧源取帧 → 跑完整流水线 → 存最新结果
    void detectLoop();

    LightbarDetector lightbarDetector_;
    NumberClassifier classifier_;
    PnpSolver pnpSolver_;

    FrameSource source_;
    std::thread thread_;

    // 最新识别结果槽：后台线程 publish、主线程 latest() 取
    LatestSlot<DetectionResult> slot_;

    // 完整识别流水线：预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP
    // 入参 FrameInput 带图像+时间戳+IMU 四元数；返回 DetectionResult，把时间戳/四元数原样透传给追踪器
    DetectionResult detect(const FrameInput& input);
};

} // namespace auto_aim
