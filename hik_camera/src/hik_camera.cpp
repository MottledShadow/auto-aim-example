#include "hik_camera.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace auto_aim::hik_camera
{
namespace
{

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

}

HikCamera::HikCamera()
{
    auto check = [&](int code, const char* step) {
        if (code != MV_OK)
        {
            cleanup();
            std::ostringstream oss;
            oss << "HikCamera " << step << " failed: 0x" << std::hex << static_cast<unsigned int>(code);
            throw std::runtime_error(oss.str());
        }
    };

    //初始化SDK
    check(MV_CC_Initialize(), "MV_CC_Initialize");
    sdkInitialized_ = true;

    //枚举相机
    MV_CC_DEVICE_INFO_LIST deviceList{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList), "MV_CC_EnumDevices");
    if (deviceList.nDeviceNum == 0 || deviceList.pDeviceInfo[0] == nullptr)
    {
        check(MV_E_NODATA, "EnumDevices(no device)");
    }

    //创建相机实例
    check(MV_CC_CreateHandle(&handle_, deviceList.pDeviceInfo[0]), "MV_CC_CreateHandle");

    //打开相机
    check(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    deviceOpened_ = true;

    //设置参数
    check(MV_CC_SetEnumValue(handle_, "ExposureAuto", 0), "SetEnumValue(ExposureAuto)");
    check(
        MV_CC_SetFloatValue(handle_, "ExposureTime", cameraOptions_.exposureTimeUs),
        "SetFloatValue(ExposureTime)");
    check(MV_CC_SetEnumValue(handle_, "TriggerMode", 0), "SetEnumValue(TriggerMode)");
    check(MV_CC_SetImageNodeNum(handle_, cameraOptions_.nodeCount), "MV_CC_SetImageNodeNum");
    check(MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly), "MV_CC_SetGrabStrategy");

    //开始取流
    check(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;

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
        check(MV_E_RESOURCE_THREAD, "std::thread(captureLoop)");
    }
}

HikCamera::~HikCamera()
{
    cleanup();
}

void HikCamera::cleanup()
{
    stopCapture_ = true;
    frameReady_.notify_all();
    if (captureThread_.joinable())
    {
        captureThread_.join();
    }
    if (grabbing_)
    {
        MV_CC_StopGrabbing(handle_);
        grabbing_ = false;
    }
    if (deviceOpened_)
    {
        MV_CC_CloseDevice(handle_);
        deviceOpened_ = false;
    }
    if (handle_ != nullptr)
    {
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    if (sdkInitialized_)
    {
        MV_CC_Finalize();
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

int HikCamera::grabFrame()
{
    //通过内部缓存获取图像
    MV_FRAME_OUT source{};
    int result = MV_CC_GetImageBuffer(handle_, &source, cameraOptions_.captureThreadTimeoutMs);
    if (result != MV_OK)
    {
        readyFrame_ = {};
        return result;
    }

    //图像处理，存info
    result = convertToBgr(handle_, source, readyFrame_.image);
    if (result == MV_OK)
    {
        const MV_FRAME_OUT_INFO_EX& info = source.stFrameInfo;
        readyFrame_.frameNumber = info.nFrameNum;
        const std::uint64_t hardwareTimestamp =
            (static_cast<std::uint64_t>(info.nDevTimeStampHigh) << 32U) |
            static_cast<std::uint64_t>(info.nDevTimeStampLow);
        readyFrame_.timestampNs = static_cast<std::uint64_t>(
            std::llround(static_cast<double>(hardwareTimestamp) * tickToNanoseconds_));
    }
    else
    {
        readyFrame_ = {};
    }

    //释放图像缓存
    const int freeResult = MV_CC_FreeImageBuffer(handle_, &source);
    return result != MV_OK ? result : freeResult;
}

void HikCamera::captureLoop()
{
    while (!stopCapture_)
    {
        int result = MV_OK;
        try
        {
            result = grabFrame();
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
            std::swap(latestFrame_, readyFrame_);
            ++publishedFrame_;
        }
        frameReady_.notify_one();
    }
}

}
