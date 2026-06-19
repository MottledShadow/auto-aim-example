#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

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
    unsigned int index = 0;
    unsigned int frames = 1;
    int timeout_ms = 1000;
    std::string output_dir = "captures";
    bool list_only = false;
    bool save = true;
    bool show = false;
    bool show_binary = false;
};

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
        << "  --help                    show this help\n\n"
        << "Examples:\n"
        << "  " << exe << " --list\n"
        << "  " << exe << " --index 0 --frames 10 --output captures\n"
        << "  " << exe << " --index 0 --frames 0 --show --no-save\n";
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
    if (std::filesystem::exists(path) && !std::filesystem::is_directory(path))
    {
        throw std::runtime_error("output path exists but is not a directory: " + path);
    }
    std::filesystem::create_directories(path);
}

std::string framePath(const std::string& output_dir, unsigned int saved_index)
{
    std::ostringstream name;
    name << "frame_" << std::setw(6) << std::setfill('0') << saved_index << ".png";
    return (std::filesystem::path(output_dir) / name.str()).string();
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
    auto_aim::HikCapture camera;
    const auto& devices = camera.devices();
    if (devices.empty())
    {
        std::cerr << "no Hikrobot camera found\n";
        return 2;
    }

    printDevices(devices);
    if (options.list_only)
    {
        return 0;
    }
    if (options.index >= devices.size())
    {
        std::cerr << "camera index out of range: " << options.index << '\n';
        return 2;
    }
    if (options.save)
    {
        ensureDirectory(options.output_dir);
    }

    camera.open(options.index, {});
    const auto_aim::Calibration calibration = auto_aim::loadCalibration();

    unsigned int saved = 0;
    while (!g_stop && (options.frames == 0 || saved < options.frames))
    {
        auto_aim::HikFrame frame;
        if (!camera.grab(frame, options.timeout_ms))
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
                  << " lights=" << result.light_bars.candidates.size()
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

        if (options.save)
        {
            const std::string path = framePath(options.output_dir, saved);
            if (!cv::imwrite(path, frame.image))
            {
                throw std::runtime_error("cv::imwrite failed: " + path);
            }
            std::cout << " saved=" << path;
        }
        std::cout << '\n';

        if (options.show)
        {
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
