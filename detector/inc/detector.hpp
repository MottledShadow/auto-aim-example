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

namespace auto_aim::detector
{

// 填好一帧 FrameInput 返回 true；暂时没帧返回 false
using FrameSource = std::function<bool(FrameInput&)>;

// 构造时起后台线程持续跑流水线、发布最新结果，主线程 latest() 取。
class Detector
{
public:
    explicit Detector(FrameSource source);
    ~Detector();

    // 取最近一帧识别结果
    DetectionResult latest();

    // 分类模型 + 标定加载失败原因，空表示都成功
    std::string error() const
    {
        std::string merged = classifier_.error();
        if (!pnpSolver_.error().empty())
        {
            if (!merged.empty())
            {
                merged += "; ";
            }
            merged += pnpSolver_.error();
        }
        return merged;
    }

private:
    void detectLoop();

    LightbarDetector lightbarDetector_;
    NumberClassifier classifier_;
    PnpSolver pnpSolver_;

    FrameSource source_;
    std::thread thread_;

    LatestSlot<DetectionResult> slot_;

    DetectionResult detect(const FrameInput& input);
};

} // namespace auto_aim::detector
