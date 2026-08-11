#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "debug_draw.hpp"
#include "detector.hpp"
#include "hik_camera.hpp"
#include "latest_slot.hpp"

namespace
{

//预处理线程交给显示线程的东西：原图 + 两种方法的检测结果（binary + candidates），绘图交给显示线程做
struct FrameResult
{
    cv::Mat frame;
    auto_aim::PreprocessResult processed_gray;
    auto_aim::PreprocessResult processed_channel;
};

int run()
{
    //初始化相机（打开设备并启动内部采集线程）
    auto_aim::HikCamera camera;
    int result = camera.initialize();
    if (result != MV_OK)
    {
        std::cerr << "initialize failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 1;
    }

    //两级流水线各一个最新帧槽：相机原图 → 预处理产出（原图+检测结果，绘图留给显示线程）
    auto_aim::LatestSlot<cv::Mat> slotRaw;
    auto_aim::LatestSlot<FrameResult> slotVis;

    //帧率计数：取帧线程累加 captureCount，预处理线程累加 detectCount，主线程按秒读差值
    std::atomic<std::uint64_t> captureCount{0};
    std::atomic<std::uint64_t> detectCount{0};

    //取帧线程：循环取最新一帧 BGR 图，发布到 slotRaw
    std::thread captureThread([&]
    {
        auto_aim::HikCameraFrame frame;
        while (slotRaw.running)
        {
            const int captureResult = camera.capture(frame);
            if (captureResult != MV_OK)
            {
                std::cerr << "capture failed: 0x"
                          << std::hex << static_cast<unsigned int>(captureResult) << '\n';
                break;
            }
            slotRaw.publish(frame.image.clone());
            ++captureCount;
        }
        //取帧结束，通知下游别再等了
        slotVis.stop();
    });

    //预处理线程：只跑 preprocess()，把原图+检测结果发给显示线程（绘图/缩放不在这里做）
    std::thread preprocessThread([&]
    {
        std::uint64_t consumed = 0;
        cv::Mat frame;
        while (slotRaw.wait(consumed, frame))
        {
            FrameResult output;
            output.frame = frame;
            //两种方法各跑一次：左panel灰度法，右panel通道相减法（默认目标红）
            auto_aim::PreprocessParams gray_params;
            gray_params.method = auto_aim::BinaryMethod::Gray;
            auto_aim::PreprocessParams channel_params;   //method 默认即 ChannelSubtract、target_color 默认 Red
            output.processed_gray = auto_aim::preprocess(frame, gray_params);
            output.processed_channel = auto_aim::preprocess(frame, channel_params);
            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //主线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，绘图/缩放也放这里
    std::cout << "一个窗口：左灰度阈值二值化，右红蓝通道相减二值化；q/ESC 退出\n";
    std::uint64_t consumed = 0;
    FrameResult result_frame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    while (slotVis.wait(consumed, result_frame))
    {
        //方法一：灰度阈值二值图（preprocess 的产出），转 BGR 方便并排和加标注
        cv::Mat panel_gray;
        cv::cvtColor(result_frame.processed_gray.binary, panel_gray, cv::COLOR_GRAY2BGR);

        //方法二：红蓝通道相减二值图（preprocess 的产出，默认目标红），转 BGR
        cv::Mat panel_channel;
        cv::cvtColor(result_frame.processed_channel.binary, panel_channel, cv::COLOR_GRAY2BGR);

        //左右各打一个方法名标注
        cv::putText(panel_gray, "gray thresh", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(panel_channel, "R-B (red)", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        //左右并排成一张
        cv::Mat combined;
        cv::hconcat(panel_gray, panel_channel, combined);

        //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比）
        auto_aim::fitToScreen(combined);

        cv::imshow("binarize compare", combined);

        //每满一秒打印一次取帧 FPS 和检测 FPS
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << (capture - lastCapture) / elapsed
                      << " detect_fps=" << (detect - lastDetect) / elapsed << '\n';
            lastPrint = now;
            lastCapture = capture;
            lastDetect = detect;
        }

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
    }

    //收尾：置停止 → 唤醒并 join 两个线程 → 关窗口和相机
    slotRaw.stop();
    slotVis.stop();
    captureThread.join();
    preprocessThread.join();
    cv::destroyAllWindows();

    result = camera.shutdown();
    if (result != MV_OK)
    {
        std::cerr << "shutdown failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 2;
    }
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
        return 3;
    }
}
