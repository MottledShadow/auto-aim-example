#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

#include "MvCameraControl.h"

namespace auto_aim
{

inline constexpr unsigned int kDefaultHikImageNodeCount = 5;
inline constexpr unsigned int kDefaultHikFrameTimeoutMs = 1000;

struct HikCameraFrame
{
    cv::Mat image;
    unsigned int frameNumber = 0;
    std::uint64_t hardwareTimestamp = 0;
    int pixelType = 0;
};

class HikCamera
{
public:
    HikCamera() = default;
    ~HikCamera();

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    int initialize(unsigned int nodeCount = kDefaultHikImageNodeCount);
    int capture(
        HikCameraFrame& frame,
        unsigned int timeoutMs = kDefaultHikFrameTimeoutMs);
    int shutdown();

private:
    void* handle_ = nullptr;
    bool sdkInitialized_ = false;
    bool deviceOpened_ = false;
    bool grabbing_ = false;
};

} // namespace auto_aim
