#include "hik_capture.hpp"

#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "MvCameraControl.h"

namespace auto_aim
{
namespace
{

std::string retHex(int ret)
{
    std::ostringstream os;
    os << "0x" << std::hex << std::uppercase << ret;
    return os.str();
}

void throwOnError(int ret, const std::string& what)
{
    if (ret != MV_OK)
    {
        throw std::runtime_error(what + " failed, ret=" + retHex(ret));
    }
}

void warnOnError(int ret, const std::string& what)
{
    if (ret != MV_OK)
    {
        std::cerr << "warning: " << what << " failed, ret=" << retHex(ret) << '\n';
    }
}

template <typename CharT, std::size_t N>
std::string safeText(const CharT (&text)[N])
{
    const char* chars = reinterpret_cast<const char*>(text);
    std::size_t len = 0;
    while (len < N && chars[len] != '\0')
    {
        ++len;
    }
    return std::string(chars, len);
}

std::string currentIp(unsigned int ip)
{
    std::ostringstream os;
    os << ((ip & 0xff000000) >> 24) << '.'
       << ((ip & 0x00ff0000) >> 16) << '.'
       << ((ip & 0x0000ff00) >> 8) << '.'
       << (ip & 0x000000ff);
    return os.str();
}

HikDeviceInfo buildDeviceInfo(unsigned int index, MV_CC_DEVICE_INFO* info)
{
    HikDeviceInfo device;
    device.index = index;

    device.accessible = MV_CC_IsDeviceAccessible(info, MV_ACCESS_Exclusive);

    if (info->nTLayerType == MV_GIGE_DEVICE)
    {
        const auto& gige = info->SpecialInfo.stGigEInfo;
        device.transport = "GigE";
        device.model = safeText(gige.chModelName);
        device.serial = safeText(gige.chSerialNumber);
        device.name = safeText(gige.chUserDefinedName);
        device.ip = currentIp(gige.nCurrentIp);
    }
    else if (info->nTLayerType == MV_USB_DEVICE)
    {
        const auto& usb = info->SpecialInfo.stUsb3VInfo;
        device.transport = "USB3";
        device.model = safeText(usb.chModelName);
        device.serial = safeText(usb.chSerialNumber);
        device.name = safeText(usb.chUserDefinedName);
    }
    else
    {
        device.transport = "Unsupported";
    }

    return device;
}

cv::Mat convertFrameToBgrOrGray(void* handle, const MV_FRAME_OUT& frame)
{
    const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;
    const unsigned int width = info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
    const unsigned int height = info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;

    if (width == 0 || height == 0 || frame.pBufAddr == nullptr)
    {
        throw std::runtime_error("empty frame buffer");
    }

    if (info.enPixelType == PixelType_Gvsp_Mono8)
    {
        return cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC1, frame.pBufAddr).clone();
    }

    if (info.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        return cv::Mat(static_cast<int>(height), static_cast<int>(width), CV_8UC3, frame.pBufAddr).clone();
    }

    if (info.enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        cv::Mat rgb(static_cast<int>(height), static_cast<int>(width), CV_8UC3, frame.pBufAddr);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }

    std::vector<unsigned char> converted(static_cast<std::size_t>(width) * height * 3 + 2048);
    MV_CC_PIXEL_CONVERT_PARAM param{};
    param.nWidth = width;
    param.nHeight = height;
    param.pSrcData = frame.pBufAddr;
    param.nSrcDataLen = info.nFrameLenEx;
    param.enSrcPixelType = info.enPixelType;
    param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    param.pDstBuffer = converted.data();
    param.nDstBufferSize = static_cast<unsigned int>(converted.size());

    throwOnError(MV_CC_ConvertPixelType(handle, &param), "MV_CC_ConvertPixelType");

    cv::Mat bgr(static_cast<int>(height), static_cast<int>(width), CV_8UC3, converted.data());
    return bgr.clone();
}

class FrameBuffer
{
public:
    explicit FrameBuffer(void* handle) : handle_(handle)
    {
    }

    ~FrameBuffer()
    {
        release();
    }

    MV_FRAME_OUT* out()
    {
        return &frame_;
    }

    const MV_FRAME_OUT& frame() const
    {
        return frame_;
    }

    void markAcquired()
    {
        acquired_ = true;
    }

private:
    void release()
    {
        if (acquired_ && frame_.pBufAddr != nullptr)
        {
            warnOnError(MV_CC_FreeImageBuffer(handle_, &frame_), "MV_CC_FreeImageBuffer");
            acquired_ = false;
        }
    }

    void* handle_ = nullptr;
    MV_FRAME_OUT frame_{};
    bool acquired_ = false;
};

} // namespace

struct HikCapture::Impl
{
    Impl()
    {
        throwOnError(MV_CC_Initialize(), "MV_CC_Initialize");
        refreshDevices();
    }

    ~Impl()
    {
        close();
        MV_CC_Finalize();
    }

    void refreshDevices()
    {
        std::memset(&device_list, 0, sizeof(device_list));
        throwOnError(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list), "MV_CC_EnumDevices");

        devices.clear();
        devices.reserve(device_list.nDeviceNum);
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i)
        {
            devices.emplace_back(buildDeviceInfo(i, device_list.pDeviceInfo[i]));
        }
    }

    void open(unsigned int index, const HikCameraConfig& config)
    {
        if (index >= device_list.nDeviceNum)
        {
            throw std::runtime_error("camera index out of range");
        }
        MV_CC_DEVICE_INFO* selected = device_list.pDeviceInfo[index];
        if (selected == nullptr)
        {
            throw std::runtime_error("camera device info is null");
        }
        if (!MV_CC_IsDeviceAccessible(selected, MV_ACCESS_Exclusive))
        {
            throw std::runtime_error("camera is not accessible in exclusive mode");
        }

        close();
        throwOnError(MV_CC_CreateHandle(&handle, selected), "MV_CC_CreateHandle");
        throwOnError(MV_CC_OpenDevice(handle), "MV_CC_OpenDevice");
        opened = true;

        applyCameraConfig(selected, config);

        throwOnError(MV_CC_StartGrabbing(handle), "MV_CC_StartGrabbing");
        grabbing = true;
    }

    bool grab(HikFrame& output, int timeout_ms)
    {
        if (handle == nullptr || !grabbing)
        {
            throw std::runtime_error("camera is not grabbing");
        }

        FrameBuffer buffer(handle);
        const int ret = MV_CC_GetImageBuffer(handle, buffer.out(), timeout_ms);
        if (ret != MV_OK)
        {
            return false;
        }
        buffer.markAcquired();

        const MV_FRAME_OUT_INFO_EX& info = buffer.frame().stFrameInfo;
        output.image = convertFrameToBgrOrGray(handle, buffer.frame());
        output.frame_number = info.nFrameNum;
        output.hardware_timestamp =
            (static_cast<std::uint64_t>(info.nDevTimeStampHigh) << 32U) |
            static_cast<std::uint64_t>(info.nDevTimeStampLow);
        output.pixel_type = static_cast<int>(info.enPixelType);
        return true;
    }

    void close()
    {
        if (grabbing)
        {
            warnOnError(MV_CC_StopGrabbing(handle), "MV_CC_StopGrabbing");
            grabbing = false;
        }
        if (opened)
        {
            warnOnError(MV_CC_CloseDevice(handle), "MV_CC_CloseDevice");
            opened = false;
        }
        if (handle != nullptr)
        {
            warnOnError(MV_CC_DestroyHandle(handle), "MV_CC_DestroyHandle");
            handle = nullptr;
        }
    }

    void applyCameraConfig(MV_CC_DEVICE_INFO* info, const HikCameraConfig& config)
    {
        setGigEPacketSizeIfNeeded(info);

        throwOnError(MV_CC_SetEnumValue(handle, "TriggerMode", 0), "set TriggerMode=Off");
        warnOnError(MV_CC_SetBayerCvtQuality(handle, 1), "set Bayer conversion quality");

        if (config.has_width)
        {
            warnOnError(MV_CC_SetIntValue(handle, "Width", config.width), "set Width");
        }
        if (config.has_height)
        {
            warnOnError(MV_CC_SetIntValue(handle, "Height", config.height), "set Height");
        }
        if (config.has_exposure)
        {
            warnOnError(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), "set ExposureAuto=Off");
            warnOnError(MV_CC_SetFloatValue(handle, "ExposureTime", config.exposure_us), "set ExposureTime");
        }
        if (config.has_gain)
        {
            warnOnError(MV_CC_SetEnumValue(handle, "GainAuto", 0), "set GainAuto=Off");
            warnOnError(MV_CC_SetFloatValue(handle, "Gain", config.gain), "set Gain");
        }
    }

    void setGigEPacketSizeIfNeeded(MV_CC_DEVICE_INFO* info)
    {
        if (info->nTLayerType != MV_GIGE_DEVICE)
        {
            return;
        }

        const int packet_size = MV_CC_GetOptimalPacketSize(handle);
        if (packet_size > 0)
        {
            warnOnError(MV_CC_SetIntValue(handle, "GevSCPSPacketSize", packet_size), "set GevSCPSPacketSize");
        }
        else
        {
            std::cerr << "warning: MV_CC_GetOptimalPacketSize failed, ret=" << retHex(packet_size) << '\n';
        }
    }

    MV_CC_DEVICE_INFO_LIST device_list{};
    std::vector<HikDeviceInfo> devices;
    void* handle = nullptr;
    bool opened = false;
    bool grabbing = false;
};

HikCapture::HikCapture() : impl_(new Impl())
{
}

HikCapture::~HikCapture() = default;

void HikCapture::refreshDevices()
{
    impl_->refreshDevices();
}

const std::vector<HikDeviceInfo>& HikCapture::devices() const
{
    return impl_->devices;
}

void HikCapture::open(unsigned int index, const HikCameraConfig& config)
{
    impl_->open(index, config);
}

bool HikCapture::grab(HikFrame& frame, int timeout_ms)
{
    return impl_->grab(frame, timeout_ms);
}

} // namespace auto_aim
