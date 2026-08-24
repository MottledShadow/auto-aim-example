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
    int pixelType = 0;
};

class HikCamera
{
public:
    // RAII：构造时跑完整条 MV_CC 初始化链并起采集线程，任一步失败即抛 std::runtime_error
    explicit HikCamera(
        unsigned int nodeCount = kDefaultHikImageNodeCount,
        float exposureTimeUs = kDefaultHikExposureTimeUs);
    ~HikCamera();

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    // 取一帧，逐帧返回 MV 码（MV_OK / 超时 MV_E_NODATA / 采集线程已出错）
    int capture(
        HikCameraFrame& frame,
        unsigned int timeoutMs = kDefaultHikFrameTimeoutMs);

private:
    // 停线程 + 逐级回收句柄/设备/SDK，构造失败回滚与析构共用
    void cleanup();
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
