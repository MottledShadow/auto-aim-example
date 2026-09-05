#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

#include "MvCameraControl.h"

namespace auto_aim::hik_camera
{

struct HikCameraOptions
{
    unsigned int nodeCount = 5;                 //SDK内部缓存节点个数
    float exposureTimeUs = 6000.0F;             //曝光时间
    unsigned int captureThreadTimeoutMs = 100;  //等待超时时间
};

struct HikCameraFrame
{
    cv::Mat image;                  //图像
    unsigned int frameNumber = 0;   //帧数
    std::uint64_t timestampNs = 0;  //已经转换的时间戳
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
    int grabFrame();
    void captureLoop();

    HikCameraOptions cameraOptions_;
    void* handle_ = nullptr;
    bool sdkInitialized_ = false;
    bool deviceOpened_ = false;
    bool grabbing_ = false;
    std::thread captureThread_;
    std::atomic_bool stopCapture_{false};
    std::mutex frameMutex_;
    std::condition_variable frameReady_;
    HikCameraFrame readyFrame_;
    HikCameraFrame latestFrame_;
    std::uint64_t publishedFrame_ = 0;
    std::uint64_t consumedFrame_ = 0;
    int captureResult_ = MV_OK;
    double tickToNanoseconds_ = 10.0;
};

}
