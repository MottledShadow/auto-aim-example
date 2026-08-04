#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>

#include <opencv2/highgui.hpp>

#include "hik_camera.hpp"

namespace
{

constexpr unsigned int kTestFrameCount = 300;

int run()
{
    auto_aim::HikCamera camera;
    int result = camera.initialize();
    if (result != MV_OK)
    {
        std::cerr << "initialize failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 1;
    }

    unsigned int frameCount = 0;
    const auto start = std::chrono::steady_clock::now();

    while (frameCount < kTestFrameCount)
    {
        auto_aim::HikCameraFrame frame;
        result = camera.capture(frame);
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

        ++frameCount;
        if (frameCount == 1 || frameCount % 30 == 0)
        {
            std::cout << "count=" << frameCount
                      << " frame=" << frame.frameNumber
                      << " timestamp=" << frame.hardwareTimestamp
                      << " size=" << frame.image.cols << 'x' << frame.image.rows
                      << " pixelType=0x" << std::hex << frame.pixelType
                      << std::dec << '\n';
        }

        cv::imshow("hik_camera_test", frame.image);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            break;
        }
    }

    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start)
                               .count();
    cv::destroyAllWindows();

    result = camera.shutdown();
    if (result != MV_OK)
    {
        std::cerr << "shutdown failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 4;
    }

    std::cout << std::fixed << std::setprecision(2)
              << "captured=" << frameCount
              << " elapsed=" << seconds << "s"
              << " average_fps=" << frameCount / seconds << '\n';
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
        return 5;
    }
}
