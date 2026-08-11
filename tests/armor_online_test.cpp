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

//处理线程交给显示线程的东西：原图 + 筛选通过的灯条 + 配对装甲数，绘图交给显示线程做
struct FrameResult
{
    cv::Mat frame;
    std::vector<auto_aim::LightBar> bars;
    std::size_t matched = 0;
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

    //两级流水线各一个最新帧槽：相机原图 → 处理产出（原图+灯条，绘图留给显示线程）
    auto_aim::LatestSlot<cv::Mat> slotRaw;
    auto_aim::LatestSlot<FrameResult> slotVis;

    //帧率计数：取帧线程累加 captureCount，处理线程累加 detectCount，主线程按秒读差值
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

    //处理线程：跑 preprocess → filterLightBars → matchArmors（都走头文件默认参数），把原图+灯条+配对数发给显示线程（绘图不在这里做）
    std::thread processThread([&]
    {
        std::uint64_t consumed = 0;
        cv::Mat frame;
        while (slotRaw.wait(consumed, frame))
        {
            //各阶段参数都走头文件默认值，要改测灰度或红色改 detector.hpp
            auto_aim::PreprocessParams pre_params;
            auto_aim::LightBarFilterParams filter_params;
            auto_aim::LightBarMatcherParams matcher_params;

            const auto_aim::PreprocessResult processed = auto_aim::preprocess(frame, pre_params);
            const std::vector<auto_aim::LightBar> bars =
                auto_aim::filterLightBars(frame, processed, filter_params);
            const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);

            FrameResult output;
            output.frame = frame;
            output.bars = bars;
            output.matched = armors.size();
            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //主线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，标注/缩放也放这里
    std::cout << "一个窗口：对所有同色灯条对按配对公式画装甲框(接受绿/被拒红)并逐项标注；q/ESC 退出\n";
    //配对参数默认值，主线程重算指标画框时用（与处理线程用的默认值一致）
    const auto_aim::LightBarMatcherParams matcher_params;
    std::uint64_t consumed = 0;
    FrameResult result_frame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    while (slotVis.wait(consumed, result_frame))
    {
        //在原图上标注：遍历全部同色灯条对，按 matchArmors 的公式重算配对指标并画框+逐项着色，返回被接受装甲数
        cv::Mat vis = result_frame.frame.clone();
        const std::size_t draw_passed = auto_aim::drawArmorMetrics(vis, result_frame.bars, matcher_params);

        //左上角标一行说明：配对装甲数（draw_passed 与处理线程的 matched 应一致，是一致性自检）
        cv::putText(vis, "armor match (default params)  matched=" + std::to_string(result_frame.matched)
                        + " draw_passed=" + std::to_string(draw_passed),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比）
        auto_aim::fitToScreen(vis);

        cv::imshow("armor match", vis);

        //每满一秒打印一次取帧 FPS、检测 FPS 和当前配对数
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << (capture - lastCapture) / elapsed
                      << " detect_fps=" << (detect - lastDetect) / elapsed
                      << " matched=" << result_frame.matched << '\n';
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
    processThread.join();
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
