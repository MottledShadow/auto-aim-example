#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

#include "MvCameraControl.h"

namespace auto_aim
{

inline constexpr unsigned int kDefaultHikImageNodeCount = 5;
inline constexpr unsigned int kDefaultHikFrameTimeoutMs = 1000;
inline constexpr float kDefaultHikExposureTimeUs = 6000.0F;

struct HikCameraFrame
{
    cv::Mat image;
    unsigned int frameNumber = 0;
    std::uint64_t hardwareTimestamp = 0;
    std::int64_t hostTimestamp = 0;   // 主机时间戳, ms
    int pixelType = 0;
};

class HikCamera
{
public:
    HikCamera() = default;
    ~HikCamera();

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    int initialize(
        unsigned int nodeCount = kDefaultHikImageNodeCount,
        float exposureTimeUs = kDefaultHikExposureTimeUs);
    int capture(
        HikCameraFrame& frame,
        unsigned int timeoutMs = kDefaultHikFrameTimeoutMs);
    int shutdown();

private:
    int grabFrame(HikCameraFrame& frame, unsigned int timeoutMs);
    void captureLoop();

    void* handle_ = nullptr;
    bool sdkInitialized_ = false;
    bool deviceOpened_ = false;
    bool grabbing_ = false;
    std::thread captureThread_;
    std::atomic_bool stopCapture_{false};
    std::mutex frameMutex_;
    std::condition_variable frameReady_;
    HikCameraFrame latestFrame_;
    std::uint64_t publishedFrame_ = 0;
    std::uint64_t consumedFrame_ = 0;
    int captureResult_ = MV_OK;
};

} // namespace auto_aim
