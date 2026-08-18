#include "hik_camera.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <system_error>
#include <utility>

namespace auto_aim
{
namespace
{

constexpr unsigned int kCaptureThreadTimeoutMs = 100;

int convertToBgr(void* handle, const MV_FRAME_OUT& source, cv::Mat& destination)
{
    const MV_FRAME_OUT_INFO_EX& info = source.stFrameInfo;
    const unsigned int width = info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
    const unsigned int height = info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;

    if (source.pBufAddr == nullptr || width == 0 || height == 0)
    {
        return MV_E_PARAMETER;
    }

    if (info.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        cv::Mat(
            static_cast<int>(height),
            static_cast<int>(width),
            CV_8UC3,
            source.pBufAddr)
            .copyTo(destination);
        return MV_OK;
    }

    destination.create(static_cast<int>(height), static_cast<int>(width), CV_8UC3);

    MV_CC_PIXEL_CONVERT_PARAM_EX parameters{};
    parameters.nWidth = width;
    parameters.nHeight = height;
    parameters.enSrcPixelType = info.enPixelType;
    parameters.pSrcData = source.pBufAddr;
    parameters.nSrcDataLen = info.nFrameLen;
    parameters.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    parameters.pDstBuffer = destination.data;
    parameters.nDstBufferSize =
        static_cast<unsigned int>(destination.total() * destination.elemSize());

    return MV_CC_ConvertPixelTypeEx(handle, &parameters);
}

} // namespace

HikCamera::~HikCamera()
{
    shutdown();
}

int HikCamera::initialize(unsigned int nodeCount, float exposureTimeUs)
{
    if (sdkInitialized_ || handle_ != nullptr)
    {
        return MV_E_CALLORDER;
    }

    int result = MV_CC_Initialize();
    if (result != MV_OK)
    {
        return result;
    }
    sdkInitialized_ = true;

    MV_CC_DEVICE_INFO_LIST deviceList{};
    result = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
    if (result == MV_OK &&
        (deviceList.nDeviceNum == 0 || deviceList.pDeviceInfo[0] == nullptr))
    {
        result = MV_E_NODATA;
    }
    if (result == MV_OK)
    {
        result = MV_CC_CreateHandle(&handle_, deviceList.pDeviceInfo[0]);
    }
    if (result == MV_OK)
    {
        result = MV_CC_OpenDevice(handle_);
        deviceOpened_ = result == MV_OK;
    }
    if (result == MV_OK)
    {
        result = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
    }
    if (result == MV_OK)
    {
        result = MV_CC_SetFloatValue(handle_, "ExposureTime", exposureTimeUs);
    }
    if (result == MV_OK)
    {
        result = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    }
    if (result == MV_OK)
    {
        result = MV_CC_SetImageNodeNum(handle_, nodeCount);
    }
    if (result == MV_OK)
    {
        result = MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly);
    }
    if (result == MV_OK)
    {
        result = MV_CC_StartGrabbing(handle_);
        grabbing_ = result == MV_OK;
    }
    if (result == MV_OK)
    {
        stopCapture_ = false;
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            latestFrame_ = {};
            publishedFrame_ = 0;
            consumedFrame_ = 0;
            captureResult_ = MV_OK;
        }
        try
        {
            captureThread_ = std::thread(&HikCamera::captureLoop, this);
        }
        catch (const std::system_error&)
        {
            result = MV_E_RESOURCE_THREAD;
        }
    }

    if (result != MV_OK)
    {
        shutdown();
    }
    return result;
}

int HikCamera::capture(HikCameraFrame& frame, unsigned int timeoutMs)
{
    if (!grabbing_ || !captureThread_.joinable())
    {
        frame = {};
        return MV_E_CALLORDER;
    }

    std::unique_lock<std::mutex> lock(frameMutex_);
    const bool ready = frameReady_.wait_for(
        lock,
        std::chrono::milliseconds(timeoutMs),
        [this]
        {
            return publishedFrame_ != consumedFrame_ ||
                   captureResult_ != MV_OK || stopCapture_;
        });
    if (!ready)
    {
        frame = {};
        return MV_E_NODATA;
    }
    if (publishedFrame_ != consumedFrame_)
    {
        std::swap(frame, latestFrame_);
        consumedFrame_ = publishedFrame_;
        return MV_OK;
    }

    frame = {};
    return captureResult_ != MV_OK ? captureResult_ : MV_E_CALLORDER;
}

int HikCamera::grabFrame(HikCameraFrame& frame, unsigned int timeoutMs)
{
    MV_FRAME_OUT source{};
    int result = MV_CC_GetImageBuffer(handle_, &source, timeoutMs);
    if (result != MV_OK)
    {
        frame = {};
        return result;
    }

    result = convertToBgr(handle_, source, frame.image);
    if (result == MV_OK)
    {
        const MV_FRAME_OUT_INFO_EX& info = source.stFrameInfo;
        frame.frameNumber = info.nFrameNum;
        frame.hardwareTimestamp =
            (static_cast<std::uint64_t>(info.nDevTimeStampHigh) << 32U) |
            static_cast<std::uint64_t>(info.nDevTimeStampLow);
        frame.hostTimestamp = info.nHostTimeStamp;
        frame.pixelType = static_cast<int>(info.enPixelType);
    }
    else
    {
        frame = {};
    }

    const int freeResult = MV_CC_FreeImageBuffer(handle_, &source);
    return result != MV_OK ? result : freeResult;
}

void HikCamera::captureLoop()
{
    HikCameraFrame frame;
    while (!stopCapture_)
    {
        int result = MV_OK;
        try
        {
            result = grabFrame(frame, kCaptureThreadTimeoutMs);
        }
        catch (const std::exception&)
        {
            result = MV_E_RESOURCE;
        }
        if (result == static_cast<int>(MV_E_NODATA))
        {
            continue;
        }
        if (result != MV_OK)
        {
            {
                std::lock_guard<std::mutex> lock(frameMutex_);
                captureResult_ = result;
            }
            frameReady_.notify_all();
            return;
        }

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            std::swap(latestFrame_, frame);
            ++publishedFrame_;
        }
        frameReady_.notify_one();
    }
}

int HikCamera::shutdown()
{
    int result = MV_OK;

    stopCapture_ = true;
    frameReady_.notify_all();
    if (captureThread_.joinable())
    {
        captureThread_.join();
    }
    if (grabbing_)
    {
        const int current = MV_CC_StopGrabbing(handle_);
        if (result == MV_OK)
        {
            result = current;
        }
        grabbing_ = false;
    }
    if (deviceOpened_)
    {
        const int current = MV_CC_CloseDevice(handle_);
        if (result == MV_OK)
        {
            result = current;
        }
        deviceOpened_ = false;
    }
    if (handle_ != nullptr)
    {
        const int current = MV_CC_DestroyHandle(handle_);
        if (result == MV_OK)
        {
            result = current;
        }
        handle_ = nullptr;
    }
    if (sdkInitialized_)
    {
        const int current = MV_CC_Finalize();
        if (result == MV_OK)
        {
            result = current;
        }
        sdkInitialized_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        latestFrame_ = {};
        publishedFrame_ = 0;
        consumedFrame_ = 0;
        captureResult_ = MV_OK;
    }
    stopCapture_ = false;

    return result;
}

} // namespace auto_aim
