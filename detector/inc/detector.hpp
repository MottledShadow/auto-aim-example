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

using FrameSource = std::function<bool(FrameInput&)>;

class Detector
{
public:
    explicit Detector(FrameSource source);
    ~Detector();

    DetectionResult latest();

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

}
