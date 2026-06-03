#include <atomic>
#include <cerrno>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "app_options.hpp"
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

std::string framePath(const std::string& output_dir, unsigned int saved_index)
{
    std::ostringstream os;
    os << output_dir << "/frame_" << std::setw(6) << std::setfill('0') << saved_index << ".png";
    return os.str();
}

std::string pixelTypeHex(int pixel_type)
{
    std::ostringstream os;
    os << "0x" << std::hex << std::uppercase << pixel_type;
    return os.str();
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

int run(const auto_aim::AppOptions& options)
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

    auto_aim::VisionPipelineParams pipeline_params;
    pipeline_params.preprocess = options.preprocess;
    pipeline_params.light_filter = options.light_filter;
    pipeline_params.armor_matcher = options.armor_matcher;
    pipeline_params.pnp = options.pnp;

    camera.open(options.index, options.camera);
    auto_aim::VisionPipeline pipeline(pipeline_params);

    unsigned int saved = 0;
    while (!g_stop && (options.frames == 0 || saved < options.frames))
    {
        auto_aim::HikFrame frame;
        if (!camera.grab(frame, options.timeout_ms))
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        const auto_aim::VisionPipelineResult result = pipeline.process(frame.image);
        std::cout << "frame=" << frame.frame_number
                  << " hardwareTimestamp=" << frame.hardware_timestamp
                  << " size=" << frame.image.cols << 'x' << frame.image.rows
                  << " channels=" << frame.image.channels()
                  << " pixelType=" << pixelTypeHex(frame.pixel_type)
                  << " contours=" << result.preprocess.contours.size()
                  << " rects=" << result.preprocess.candidate_rects.size()
                  << " lines=" << result.preprocess.candidate_center_lines.size()
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
        const auto_aim::AppOptions options = auto_aim::parseArgs(argc, argv);
        return run(options);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << "\n\n";
        auto_aim::printUsage(argv[0]);
        return 1;
    }
}
