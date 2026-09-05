#include "detector.hpp"

#include <stdexcept>
#include <utility>

namespace auto_aim::detector
{

namespace
{
constexpr int kPnPInputMode = 0;
}

Detector::Detector(FrameSource source)
    : classifier_(NumberClassifierParams{})
    , pnpSolver_(PnpSolverParams{})
    , source_(std::move(source))
{
    if (!error().empty())
    {
        throw std::runtime_error("Detector init failed: " + error());
    }

    thread_ = std::thread(&Detector::detectLoop, this);
}

Detector::~Detector()
{
    slot_.stop();
    if (thread_.joinable())
    {
        thread_.join();
    }
}

DetectionResult Detector::detect(const FrameInput& input)
{
    const PreprocessResult pre = lightbarDetector_.preprocess(input.image);

    const std::vector<LightBar> bars = lightbarDetector_.filterLightBars(input.image, pre);

    const std::vector<Armor> armors = lightbarDetector_.matchArmors(bars);

    const std::vector<Armor> classified = classifier_.classify(input.image, armors);

    const std::vector<Armor> solved =
        kPnPInputMode == 0 ? pnpSolver_.solve(armors) : pnpSolver_.solve(classified);

    return DetectionResult{solved, input.timestampNs, input.quaternion};
}

void Detector::detectLoop()
{
    while (slot_.running)
    {
        FrameInput input;
        if (!source_(input))
        {
            continue;
        }

        DetectionResult result = detect(input);

        slot_.publish(std::move(result));
    }
}

DetectionResult Detector::latest()
{
    return slot_.latest();
}

}
