#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

#include <opencv2/core.hpp>

#include "MvCameraControl.h"

namespace auto_aim::hik_camera
{

struct HikCameraOptions
{
    unsigned int nodeCount = 5;
    float exposureTimeUs = 6000.0F;
    unsigned int captureThreadTimeoutMs = 100;
};

struct HikCameraFrame
{
    cv::Mat image;
    unsigned int frameNumber = 0;
    std::uint64_t hardwareTimestamp = 0;        //相机时间戳
    std::optional<std::uint64_t> timestampNs;   //标定后的时间戳
};

class HikCamera
{
public:
    HikCamera();
    ~HikCamera();

    HikCamera(const HikCamera&) = delete;
    HikCamera& operator=(const HikCamera&) = delete;

    int capture(
        HikCameraFrame& frame,
        unsigned int timeoutMs = 1000);

private:
    void cleanup();
    int grabFrame(HikCameraFrame& frame);
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
    HikCameraOptions cameraOptions_;
    double tickToNanoseconds_ = 10.0;
};

}
