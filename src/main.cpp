#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>

#include "debug_draw.hpp"
#include "hik_capture.hpp"
#include "vision_pipeline.hpp"

namespace
{

std::atomic_bool g_stop{false};

void handleSignal(int)
{
    g_stop = true;
}

struct AppOptions
{
    bool show_binary = false;
};

void printUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [--show-binary] [--help]\n\n"
        << "Grab frames from the first Hikrobot camera and show a live debug preview.\n"
        << "Press q or Esc to quit.\n\n"
        << "Options:\n"
        << "  --show-binary   show preprocessed binary image instead of raw preview\n"
        << "  --help          show this help\n";
}

AppOptions parseArgs(int argc, char** argv)
{
    AppOptions options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        else if (arg == "--show-binary")
        {
            options.show_binary = true;
        }
        else
        {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    return options;
}

void printDevices(const std::vector<auto_aim::HikDeviceInfo>& devices)
{
    for (const auto& device : devices)
    {
        std::cout << '[' << device.index << "] "
                  << device.transport << "  "
                  << "model=" << device.model << "  "
                  << "serial=" << device.serial << "  "
                  << "name=" << device.name;
        if (!device.ip.empty())
        {
            std::cout << "  ip=" << device.ip;
        }
        std::cout << "  accessible=" << (device.accessible ? "yes" : "no") << '\n';
    }
}

int run(const AppOptions& options)
{
    constexpr unsigned int kCameraIndex = 0;
    constexpr int kTimeoutMs = 1000;

    auto_aim::HikCapture camera;
    const auto& devices = camera.devices();
    if (devices.empty())
    {
        std::cerr << "no Hikrobot camera found\n";
        return 2;
    }

    printDevices(devices);

    camera.open(kCameraIndex, {});
    const auto_aim::Calibration calibration = auto_aim::loadCalibration();

    while (!g_stop)
    {
        auto_aim::HikFrame frame;
        if (!camera.grab(frame, kTimeoutMs))
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        const auto_aim::VisionPipelineResult result = auto_aim::runPipeline(frame.image, calibration);
        std::cout << "frame=" << frame.frame_number
                  << " hardwareTimestamp=" << frame.hardware_timestamp
                  << " size=" << frame.image.cols << 'x' << frame.image.rows
                  << " channels=" << frame.image.channels()
                  << " pixelType=0x" << std::hex << std::uppercase << frame.pixel_type
                  << std::dec << std::nouppercase
                  << " rects=" << result.preprocess.candidates.size()
                  << " lines=" << result.preprocess.candidates.size()
                  << " lights=" << result.light_bars.size()
                  << " armors=" << result.armors.candidates.size()
                  << " poses=" << result.pnp.poses.size();

        if (!result.pnp.poses.empty())
        {
            const cv::Mat& tvec = result.pnp.poses.front().tvec;
            std::cout << " first_tvec=("
                      << tvec.at<double>(0, 0) << ','
                      << tvec.at<double>(1, 0) << ','
                      << tvec.at<double>(2, 0) << ')';
        }
        else if (!result.pnp.calibration_error.empty())
        {
            std::cout << " pnp_error=" << result.pnp.calibration_error;
        }
        std::cout << '\n';

        const cv::Mat preview = auto_aim::makeDebugPreview(
            frame.image,
            result.preprocess,
            result.light_bars,
            result.armors,
            options.show_binary);
        cv::imshow("hik_capture", preview);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            break;
        }
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
        const AppOptions options = parseArgs(argc, argv);
        return run(options);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
}
