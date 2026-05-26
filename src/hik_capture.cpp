#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_matcher.hpp"
#include "armor_preprocessor.hpp"
#include "light_bar_filter.hpp"
#include "MvCameraControl.h"

namespace
{

std::atomic_bool g_stop{false};

void handleSignal(int)
{
    g_stop = true;
}

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

std::string safeText(const char* text, std::size_t max_len)
{
    std::size_t len = 0;
    while (len < max_len && text[len] != '\0')
    {
        ++len;
    }
    return std::string(text, len);
}

std::string safeText(const unsigned char* text, std::size_t max_len)
{
    return safeText(reinterpret_cast<const char*>(text), max_len);
}

template <std::size_t N>
std::string safeText(const char (&text)[N])
{
    return safeText(text, N);
}

template <std::size_t N>
std::string safeText(const unsigned char (&text)[N])
{
    return safeText(text, N);
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

void printDeviceInfo(unsigned int index, const MV_CC_DEVICE_INFO* info)
{
    if (info == nullptr)
    {
        std::cout << '[' << index << "] null device info\n";
        return;
    }

    std::cout << '[' << index << "] ";
    if (info->nTLayerType == MV_GIGE_DEVICE)
    {
        const auto& gige = info->SpecialInfo.stGigEInfo;
        std::cout << "GigE  "
                  << "model=" << safeText(gige.chModelName) << "  "
                  << "serial=" << safeText(gige.chSerialNumber) << "  "
                  << "name=" << safeText(gige.chUserDefinedName) << "  "
                  << "ip=" << currentIp(gige.nCurrentIp) << '\n';
    }
    else if (info->nTLayerType == MV_USB_DEVICE)
    {
        const auto& usb = info->SpecialInfo.stUsb3VInfo;
        std::cout << "USB3  "
                  << "model=" << safeText(usb.chModelName) << "  "
                  << "serial=" << safeText(usb.chSerialNumber) << "  "
                  << "name=" << safeText(usb.chUserDefinedName) << '\n';
    }
    else
    {
        std::cout << "unsupported transport layer: " << info->nTLayerType << '\n';
    }
}

bool startsWith(const std::string& text, const std::string& prefix)
{
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

std::string takeValue(int& i, int argc, char** argv, const std::string& arg, const std::string& key)
{
    const std::string with_equals = key + "=";
    if (startsWith(arg, with_equals))
    {
        return arg.substr(with_equals.size());
    }
    if (arg == key && i + 1 < argc)
    {
        return argv[++i];
    }
    throw std::runtime_error("missing value for " + key);
}

unsigned int parseUInt(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    unsigned long parsed = std::stoul(value, &pos, 10);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid integer for " + key + ": " + value);
    }
    return static_cast<unsigned int>(parsed);
}

int parseByte(const std::string& value, const std::string& key)
{
    const unsigned int parsed = parseUInt(value, key);
    if (parsed > 255)
    {
        throw std::runtime_error("invalid threshold for " + key + ": " + value);
    }
    return static_cast<int>(parsed);
}

float parseFloat(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    float parsed = std::stof(value, &pos);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid number for " + key + ": " + value);
    }
    return parsed;
}

double parseDouble(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    double parsed = std::stod(value, &pos);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid number for " + key + ": " + value);
    }
    return parsed;
}

struct Options
{
    unsigned int index = 0;
    unsigned int frames = 1;
    int timeout_ms = 1000;
    std::string output_dir = "captures";
    bool list_only = false;
    bool save = true;
    bool show = false;
    bool show_binary = false;
    bool has_exposure = false;
    bool has_gain = false;
    bool has_width = false;
    bool has_height = false;
    auto_aim::ArmorPreprocessParams preprocess;
    auto_aim::LightBarFilterParams light_filter;
    auto_aim::ArmorMatcherParams armor_matcher;
    float exposure_us = 0.0F;
    float gain = 0.0F;
    unsigned int width = 0;
    unsigned int height = 0;
};

void printUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [options]\n\n"
        << "Options:\n"
        << "  --list                    list cameras and exit\n"
        << "  --index N                 camera index, default 0\n"
        << "  --frames N                number of frames to grab, 0 means until Ctrl-C\n"
        << "  --timeout-ms N            frame timeout, default 1000\n"
        << "  --output DIR              save PNG frames to DIR, default captures\n"
        << "  --no-save                 grab/preview without writing images\n"
        << "  --show                    show live OpenCV preview, press q or Esc to quit\n"
        << "  --show-binary             show preprocessed binary image instead of raw preview\n"
        << "  --binary-threshold N      threshold for bright region extraction, default 180\n"
        << "  --open-kernel N           opening kernel size, 0 disables opening, default 3\n"
        << "  --close-kernel N          closing kernel size, 0 disables closing, default 3\n"
        << "  --morph-iterations N      opening/closing iterations, default 1\n"
        << "  --min-light-area VALUE    minimum light bar foreground area, default 5\n"
        << "  --max-light-area VALUE    maximum light bar foreground area, default 1000000\n"
        << "  --min-light-aspect VALUE  minimum long-side/short-side ratio, default 1.2\n"
        << "  --max-light-aspect VALUE  maximum long-side/short-side ratio, default 50\n"
        << "  --min-light-angle VALUE   minimum fitLine tilt from vertical, default 0\n"
        << "  --max-light-angle VALUE   maximum fitLine tilt from vertical, default 45\n"
        << "  --min-light-fill VALUE    minimum area/minAreaRect-area ratio, default 0.25\n"
        << "  --max-light-fill VALUE    maximum area/minAreaRect-area ratio, default 1\n"
        << "  --max-armor-length-ratio VALUE  maximum paired light length ratio, default 2\n"
        << "  --max-armor-angle-diff VALUE    maximum paired light angle difference, default 10\n"
        << "  --max-armor-y-diff VALUE        maximum paired light center y difference, default 40\n"
        << "  --min-armor-distance-ratio VALUE minimum center distance / average height, default 0.5\n"
        << "  --max-armor-distance-ratio VALUE maximum center distance / average height, default 8\n"
        << "  --exposure-us VALUE       set manual exposure time in microseconds\n"
        << "  --gain VALUE              set manual gain\n"
        << "  --width N                 set camera Width before grabbing\n"
        << "  --height N                set camera Height before grabbing\n"
        << "  --help                    show this help\n\n"
        << "Examples:\n"
        << "  " << exe << " --list\n"
        << "  " << exe << " --index 0 --frames 10 --output captures\n"
        << "  " << exe << " --index 0 --frames 0 --show --no-save --exposure-us 3000\n";
}

Options parseArgs(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--list")
        {
            options.list_only = true;
        }
        else if (arg == "--no-save")
        {
            options.save = false;
        }
        else if (arg == "--show")
        {
            options.show = true;
        }
        else if (arg == "--show-binary")
        {
            options.show_binary = true;
            options.show = true;
        }
        else if (arg == "--index" || startsWith(arg, "--index="))
        {
            options.index = parseUInt(takeValue(i, argc, argv, arg, "--index"), "--index");
        }
        else if (arg == "--frames" || startsWith(arg, "--frames="))
        {
            options.frames = parseUInt(takeValue(i, argc, argv, arg, "--frames"), "--frames");
        }
        else if (arg == "--timeout-ms" || startsWith(arg, "--timeout-ms="))
        {
            options.timeout_ms = static_cast<int>(parseUInt(takeValue(i, argc, argv, arg, "--timeout-ms"), "--timeout-ms"));
        }
        else if (arg == "--output" || startsWith(arg, "--output="))
        {
            options.output_dir = takeValue(i, argc, argv, arg, "--output");
        }
        else if (arg == "--binary-threshold" || startsWith(arg, "--binary-threshold="))
        {
            options.preprocess.binary_threshold = parseByte(
                takeValue(i, argc, argv, arg, "--binary-threshold"),
                "--binary-threshold");
        }
        else if (arg == "--open-kernel" || startsWith(arg, "--open-kernel="))
        {
            options.preprocess.open_kernel_size = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--open-kernel"), "--open-kernel"));
        }
        else if (arg == "--close-kernel" || startsWith(arg, "--close-kernel="))
        {
            options.preprocess.close_kernel_size = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--close-kernel"), "--close-kernel"));
        }
        else if (arg == "--morph-iterations" || startsWith(arg, "--morph-iterations="))
        {
            options.preprocess.morph_iterations = static_cast<int>(
                parseUInt(takeValue(i, argc, argv, arg, "--morph-iterations"), "--morph-iterations"));
        }
        else if (arg == "--min-light-area" || startsWith(arg, "--min-light-area="))
        {
            options.light_filter.min_area = parseDouble(
                takeValue(i, argc, argv, arg, "--min-light-area"),
                "--min-light-area");
        }
        else if (arg == "--max-light-area" || startsWith(arg, "--max-light-area="))
        {
            options.light_filter.max_area = parseDouble(
                takeValue(i, argc, argv, arg, "--max-light-area"),
                "--max-light-area");
        }
        else if (arg == "--min-light-aspect" || startsWith(arg, "--min-light-aspect="))
        {
            options.light_filter.min_aspect_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--min-light-aspect"),
                "--min-light-aspect");
        }
        else if (arg == "--max-light-aspect" || startsWith(arg, "--max-light-aspect="))
        {
            options.light_filter.max_aspect_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-light-aspect"),
                "--max-light-aspect");
        }
        else if (arg == "--min-light-angle" || startsWith(arg, "--min-light-angle="))
        {
            options.light_filter.min_line_angle_deg = parseDouble(
                takeValue(i, argc, argv, arg, "--min-light-angle"),
                "--min-light-angle");
        }
        else if (arg == "--max-light-angle" || startsWith(arg, "--max-light-angle="))
        {
            options.light_filter.max_line_angle_deg = parseDouble(
                takeValue(i, argc, argv, arg, "--max-light-angle"),
                "--max-light-angle");
        }
        else if (arg == "--min-light-fill" || startsWith(arg, "--min-light-fill="))
        {
            options.light_filter.min_fill_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--min-light-fill"),
                "--min-light-fill");
        }
        else if (arg == "--max-light-fill" || startsWith(arg, "--max-light-fill="))
        {
            options.light_filter.max_fill_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-light-fill"),
                "--max-light-fill");
        }
        else if (arg == "--max-armor-length-ratio" || startsWith(arg, "--max-armor-length-ratio="))
        {
            options.armor_matcher.max_light_length_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-length-ratio"),
                "--max-armor-length-ratio");
        }
        else if (arg == "--max-armor-angle-diff" || startsWith(arg, "--max-armor-angle-diff="))
        {
            options.armor_matcher.max_light_angle_diff_deg = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-angle-diff"),
                "--max-armor-angle-diff");
        }
        else if (arg == "--max-armor-y-diff" || startsWith(arg, "--max-armor-y-diff="))
        {
            options.armor_matcher.max_light_center_y_diff = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-y-diff"),
                "--max-armor-y-diff");
        }
        else if (arg == "--min-armor-distance-ratio" || startsWith(arg, "--min-armor-distance-ratio="))
        {
            options.armor_matcher.min_center_distance_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--min-armor-distance-ratio"),
                "--min-armor-distance-ratio");
        }
        else if (arg == "--max-armor-distance-ratio" || startsWith(arg, "--max-armor-distance-ratio="))
        {
            options.armor_matcher.max_center_distance_ratio = parseDouble(
                takeValue(i, argc, argv, arg, "--max-armor-distance-ratio"),
                "--max-armor-distance-ratio");
        }
        else if (arg == "--exposure-us" || startsWith(arg, "--exposure-us="))
        {
            options.has_exposure = true;
            options.exposure_us = parseFloat(takeValue(i, argc, argv, arg, "--exposure-us"), "--exposure-us");
        }
        else if (arg == "--gain" || startsWith(arg, "--gain="))
        {
            options.has_gain = true;
            options.gain = parseFloat(takeValue(i, argc, argv, arg, "--gain"), "--gain");
        }
        else if (arg == "--width" || startsWith(arg, "--width="))
        {
            options.has_width = true;
            options.width = parseUInt(takeValue(i, argc, argv, arg, "--width"), "--width");
        }
        else if (arg == "--height" || startsWith(arg, "--height="))
        {
            options.has_height = true;
            options.height = parseUInt(takeValue(i, argc, argv, arg, "--height"), "--height");
        }
        else
        {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    return options;
}

void ensureDirectory(const std::string& path)
{
    if (path.empty())
    {
        throw std::runtime_error("output directory cannot be empty");
    }

    struct stat st = {};
    if (::stat(path.c_str(), &st) == 0)
    {
        if (!S_ISDIR(st.st_mode))
        {
            throw std::runtime_error("output path exists but is not a directory: " + path);
        }
        return;
    }

    if (errno != ENOENT)
    {
        throw std::runtime_error("cannot inspect output path: " + path);
    }

    if (::mkdir(path.c_str(), 0755) != 0)
    {
        throw std::runtime_error("cannot create output directory: " + path);
    }
}

unsigned int frameWidth(const MV_FRAME_OUT_INFO_EX& info)
{
    return info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
}

unsigned int frameHeight(const MV_FRAME_OUT_INFO_EX& info)
{
    return info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;
}

std::string framePath(const std::string& output_dir, unsigned int saved_index)
{
    std::ostringstream os;
    os << output_dir << "/frame_" << std::setw(6) << std::setfill('0') << saved_index << ".png";
    return os.str();
}

void drawCandidateRects(cv::Mat& image, const std::vector<cv::RotatedRect>& rects)
{
    for (const auto& rect : rects)
    {
        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
}

void drawCandidateCenterLines(
    cv::Mat& image,
    const std::vector<cv::RotatedRect>& rects,
    const std::vector<cv::Vec4f>& center_lines)
{
    const std::size_t count = std::min(rects.size(), center_lines.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        const cv::Vec4f& line = center_lines[i];
        const cv::RotatedRect& rect = rects[i];
        const cv::Point2f direction(line[0], line[1]);
        const cv::Point2f point_on_line(line[2], line[3]);
        const float half_length = std::max(rect.size.width, rect.size.height) * 0.5F;
        cv::line(
            image,
            point_on_line - direction * half_length,
            point_on_line + direction * half_length,
            cv::Scalar(0, 0, 255),
            2);
    }
}

void drawLightBars(cv::Mat& image, const std::vector<auto_aim::LightBarCandidate>& light_bars)
{
    for (const auto& candidate : light_bars)
    {
        const auto& light = candidate.light_bar;
        cv::Scalar color(0, 255, 255);
        if (light.color == auto_aim::LightColor::Red)
        {
            color = cv::Scalar(0, 0, 255);
        }
        else if (light.color == auto_aim::LightColor::Blue)
        {
            color = cv::Scalar(255, 0, 0);
        }

        cv::line(image, light.top, light.bottom, color, 3);
        cv::circle(image, light.center, 3, color, -1);
    }
}

cv::Point toImagePoint(const cv::Point2f& point)
{
    return cv::Point(cvRound(point.x), cvRound(point.y));
}

void drawArmors(cv::Mat& image, const std::vector<auto_aim::ArmorCandidate>& armors)
{
    for (const auto& candidate : armors)
    {
        const auto& armor = candidate.armor;
        std::vector<cv::Point> points{
            toImagePoint(armor.left_light.top),
            toImagePoint(armor.right_light.top),
            toImagePoint(armor.right_light.bottom),
            toImagePoint(armor.left_light.bottom),
        };
        cv::polylines(image, points, true, cv::Scalar(255, 255, 255), 2);
        cv::circle(image, toImagePoint(armor.center), 4, cv::Scalar(255, 255, 255), -1);
    }
}

class SdkGuard
{
public:
    SdkGuard()
    {
        throwOnError(MV_CC_Initialize(), "MV_CC_Initialize");
    }

    ~SdkGuard()
    {
        MV_CC_Finalize();
    }

    SdkGuard(const SdkGuard&) = delete;
    SdkGuard& operator=(const SdkGuard&) = delete;
};

class CameraHandle
{
public:
    ~CameraHandle()
    {
        stop();
        close();
        destroy();
    }

    void create(MV_CC_DEVICE_INFO* info)
    {
        throwOnError(MV_CC_CreateHandle(&handle_, info), "MV_CC_CreateHandle");
    }

    void open()
    {
        throwOnError(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
        opened_ = true;
    }

    void start()
    {
        throwOnError(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
        grabbing_ = true;
    }

    void* get() const
    {
        return handle_;
    }

private:
    void stop()
    {
        if (grabbing_)
        {
            warnOnError(MV_CC_StopGrabbing(handle_), "MV_CC_StopGrabbing");
            grabbing_ = false;
        }
    }

    void close()
    {
        if (opened_)
        {
            warnOnError(MV_CC_CloseDevice(handle_), "MV_CC_CloseDevice");
            opened_ = false;
        }
    }

    void destroy()
    {
        if (handle_ != nullptr)
        {
            warnOnError(MV_CC_DestroyHandle(handle_), "MV_CC_DestroyHandle");
            handle_ = nullptr;
        }
    }

    void* handle_ = nullptr;
    bool opened_ = false;
    bool grabbing_ = false;
};

void setGigEPacketSizeIfNeeded(void* handle, const MV_CC_DEVICE_INFO* info)
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

void applyCameraOptions(void* handle, const MV_CC_DEVICE_INFO* info, const Options& options)
{
    setGigEPacketSizeIfNeeded(handle, info);

    throwOnError(MV_CC_SetEnumValue(handle, "TriggerMode", 0), "set TriggerMode=Off");
    warnOnError(MV_CC_SetBayerCvtQuality(handle, 1), "set Bayer conversion quality");

    if (options.has_width)
    {
        warnOnError(MV_CC_SetIntValue(handle, "Width", options.width), "set Width");
    }
    if (options.has_height)
    {
        warnOnError(MV_CC_SetIntValue(handle, "Height", options.height), "set Height");
    }
    if (options.has_exposure)
    {
        warnOnError(MV_CC_SetEnumValue(handle, "ExposureAuto", 0), "set ExposureAuto=Off");
        warnOnError(MV_CC_SetFloatValue(handle, "ExposureTime", options.exposure_us), "set ExposureTime");
    }
    if (options.has_gain)
    {
        warnOnError(MV_CC_SetEnumValue(handle, "GainAuto", 0), "set GainAuto=Off");
        warnOnError(MV_CC_SetFloatValue(handle, "Gain", options.gain), "set Gain");
    }
}

class CameraSession
{
public:
    CameraSession(MV_CC_DEVICE_INFO* info, const Options& options)
    {
        camera_.create(info);
        camera_.open();
        applyCameraOptions(camera_.get(), info, options);
        camera_.start();
    }

    void* handle() const
    {
        return camera_.get();
    }

private:
    CameraHandle camera_;
};

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

cv::Mat convertFrameToBgrOrGray(void* handle, const MV_FRAME_OUT& frame)
{
    const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;
    const unsigned int width = frameWidth(info);
    const unsigned int height = frameHeight(info);

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

MV_CC_DEVICE_INFO_LIST enumDevices()
{
    MV_CC_DEVICE_INFO_LIST list{};
    throwOnError(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list), "MV_CC_EnumDevices");
    return list;
}

int run(const Options& options)
{
    SdkGuard sdk;
    MV_CC_DEVICE_INFO_LIST devices = enumDevices();

    if (devices.nDeviceNum == 0)
    {
        std::cerr << "no Hikrobot camera found\n";
        return 2;
    }

    for (unsigned int i = 0; i < devices.nDeviceNum; ++i)
    {
        printDeviceInfo(i, devices.pDeviceInfo[i]);
    }

    if (options.list_only)
    {
        return 0;
    }

    if (options.index >= devices.nDeviceNum)
    {
        std::cerr << "camera index out of range: " << options.index << '\n';
        return 2;
    }

    MV_CC_DEVICE_INFO* selected = devices.pDeviceInfo[options.index];
    if (!MV_CC_IsDeviceAccessible(selected, MV_ACCESS_Exclusive))
    {
        std::cerr << "camera " << options.index << " is not accessible in exclusive mode\n";
        return 2;
    }

    if (options.save)
    {
        ensureDirectory(options.output_dir);
    }

    CameraSession camera(selected, options);
    auto_aim::ArmorPreprocessor preprocessor(options.preprocess);
    auto_aim::LightBarFilter light_bar_filter(options.light_filter);
    auto_aim::ArmorMatcher armor_matcher(options.armor_matcher);
    unsigned int saved = 0;

    while (!g_stop && (options.frames == 0 || saved < options.frames))
    {
        FrameBuffer buffer(camera.handle());
        const int ret = MV_CC_GetImageBuffer(camera.handle(), buffer.out(), options.timeout_ms);
        if (ret != MV_OK)
        {
            std::cerr << "warning: frame timeout/error, ret=" << retHex(ret) << '\n';
            continue;
        }
        buffer.markAcquired();

        const MV_FRAME_OUT_INFO_EX& info = buffer.frame().stFrameInfo;
        cv::Mat image = convertFrameToBgrOrGray(camera.handle(), buffer.frame());
        const auto_aim::ArmorPreprocessResult preprocess = preprocessor.process(image);
        const auto_aim::LightBarFilterResult light_bars = light_bar_filter.filter(image, preprocess);
        const auto_aim::ArmorMatchResult armors = armor_matcher.match(light_bars);
        std::cout << "frame=" << info.nFrameNum
                  << " size=" << image.cols << 'x' << image.rows
                  << " channels=" << image.channels()
                  << " pixelType=" << retHex(static_cast<int>(info.enPixelType))
                  << " contours=" << preprocess.contours.size()
                  << " rects=" << preprocess.candidate_rects.size()
                  << " lines=" << preprocess.candidate_center_lines.size()
                  << " lights=" << light_bars.candidates.size()
                  << " armors=" << armors.candidates.size();

        if (options.save)
        {
            const std::string path = framePath(options.output_dir, saved);
            if (!cv::imwrite(path, image))
            {
                throw std::runtime_error("cv::imwrite failed: " + path);
            }
            std::cout << " saved=" << path;
        }
        std::cout << '\n';

        if (options.show)
        {
            cv::Mat preview;
            if (options.show_binary)
            {
                cv::cvtColor(preprocess.binary, preview, cv::COLOR_GRAY2BGR);
            }
            else if (image.channels() == 1)
            {
                cv::cvtColor(image, preview, cv::COLOR_GRAY2BGR);
            }
            else
            {
                preview = image.clone();
            }
            drawCandidateRects(preview, preprocess.candidate_rects);
            drawCandidateCenterLines(
                preview,
                preprocess.candidate_rects,
                preprocess.candidate_center_lines);
            drawLightBars(preview, light_bars.candidates);
            drawArmors(preview, armors.candidates);
            cv::imshow("hik_capture", preview);
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }
        }

        ++saved;
    }

    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try
    {
        const Options options = parseArgs(argc, argv);
        return run(options);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
}
