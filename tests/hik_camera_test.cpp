#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <set>

#include <opencv2/highgui.hpp>

#include "hik_camera.hpp"

namespace
{

constexpr unsigned int kWarmupFrameCount = 30;
constexpr unsigned int kTestFrameCount = 300;

struct PhaseStats
{
    unsigned int captured = 0;
    unsigned int firstFrame = 0;
    unsigned int lastFrame = 0;
    double elapsedSeconds = 0.0;
    double captureSeconds = 0.0;
    double displaySeconds = 0.0;
    std::set<const unsigned char*> bufferAddresses;
};

int captureFrames(
    auto_aim::HikCamera& camera,
    bool preview,
    unsigned int frameLimit,
    PhaseStats& stats)
{
    auto_aim::HikCameraFrame frame;
    const auto phaseStart = std::chrono::steady_clock::now();

    while (stats.captured < frameLimit)
    {
        const auto captureStart = std::chrono::steady_clock::now();
        const int result = camera.capture(frame);
        stats.captureSeconds += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - captureStart)
                                    .count();
        if (result != MV_OK)
        {
            std::cerr << "capture failed: 0x"
                      << std::hex << static_cast<unsigned int>(result) << '\n';
            return 2;
        }
        if (frame.image.empty() || frame.image.type() != CV_8UC3)
        {
            std::cerr << "invalid BGR image\n";
            return 3;
        }

        if (stats.captured == 0)
        {
            stats.firstFrame = frame.frameNumber;
        }
        stats.bufferAddresses.insert(frame.image.data);
        stats.lastFrame = frame.frameNumber;
        ++stats.captured;

        if (preview)
        {
            const auto displayStart = std::chrono::steady_clock::now();
            cv::imshow("hik_camera_test", frame.image);
            const int key = cv::waitKey(1);
            stats.displaySeconds += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - displayStart)
                                        .count();
            if (key == 27 || key == 'q' || key == 'Q')
            {
                break;
            }
        }
    }

    stats.elapsedSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phaseStart)
                               .count();
    if (stats.bufferAddresses.size() > 3)
    {
        std::cerr << "image buffers were reallocated during capture\n";
        return 4;
    }
    return MV_OK;
}

void printStats(const char* phase, const PhaseStats& stats)
{
    const unsigned int cameraFrames = stats.lastFrame - stats.firstFrame + 1;
    const unsigned int skippedFrames =
        cameraFrames > stats.captured ? cameraFrames - stats.captured : 0;
    const double sourceFps = cameraFrames / stats.elapsedSeconds;

    std::cout << std::fixed << std::setprecision(2)
              << "phase=" << phase
              << " captured=" << stats.captured
              << " camera_frames=" << cameraFrames
              << " skipped=" << skippedFrames
              << " unique_buffers=" << stats.bufferAddresses.size()
              << " elapsed=" << stats.elapsedSeconds << "s"
              << " throughput_fps=" << stats.captured / stats.elapsedSeconds
              << " source_fps=" << sourceFps
              << " average_capture_ms="
              << stats.captureSeconds * 1000.0 / stats.captured
              << " average_display_ms="
              << stats.displaySeconds * 1000.0 / stats.captured << '\n';
}

int run()
{
    // 构造即初始化，失败会抛异常
    auto_aim::HikCamera camera;

    PhaseStats warmup;
    int result = captureFrames(camera, false, kWarmupFrameCount, warmup);
    if (result != MV_OK)
    {
        return result;
    }

    PhaseStats headless;
    result = captureFrames(camera, false, kTestFrameCount, headless);
    if (result != MV_OK)
    {
        return result;
    }
    printStats("headless", headless);

    PhaseStats preview;
    result = captureFrames(camera, true, kTestFrameCount, preview);
    cv::destroyAllWindows();
    if (result != MV_OK)
    {
        return result;
    }
    printStats("preview", preview);

    // 相机析构时自动清理（停线程 + 回收句柄/设备/SDK）
    return 0;
}

} // namespace

int main()
{
    try
    {
        return run();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test failed: " << exception.what() << '\n';
        return 6;
    }
}
