#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
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

//处理线程交给显示线程的东西：原图 + 解算出的位姿(含 rvec/tvec) + 配对装甲数，绘图交给显示线程做
struct FrameResult
{
    cv::Mat frame;
    std::vector<auto_aim::ArmorPose> poses;
    std::size_t matched = 0;
};

int run()
{
    //先加载相机标定，失败只告警不退出（后续 solvePnp 会返回空位姿，窗口只画不出位姿）
    const auto_aim::CameraCalibration calibration = auto_aim::loadCalibration();
    if (!calibration.error.empty())
    {
        std::cerr << "calibration load failed: " << calibration.error << '\n';
    }

    //初始化相机（打开设备并启动内部采集线程）
    auto_aim::HikCamera camera;
    int result = camera.initialize();
    if (result != MV_OK)
    {
        std::cerr << "initialize failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 1;
    }

    //两级流水线各一个最新帧槽：相机原图 → 处理产出（原图+位姿，绘图留给显示线程）
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

    //处理线程：跑 preprocess → filterLightBars → matchArmors → solvePnp（都走头文件默认参数），把原图+位姿+配对数发给显示线程
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
            auto_aim::PnpSolverParams pnp_params;

            const auto_aim::PreprocessResult processed = auto_aim::preprocess(frame, pre_params);
            const std::vector<auto_aim::LightBar> bars =
                auto_aim::filterLightBars(frame, processed, filter_params);
            const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);

            FrameResult output;
            output.frame = frame;
            output.poses = auto_aim::solvePnp(armors, calibration, pnp_params);
            output.matched = armors.size();
            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //主线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，标注/缩放也放这里
    std::cout << "一个窗口：画装甲框并标注每块装甲的 rvec/tvec；q/ESC 退出\n";
    std::uint64_t consumed = 0;
    FrameResult result_frame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    while (slotVis.wait(consumed, result_frame))
    {
        //在原图上逐位姿画装甲四边形并标注 rvec/tvec
        cv::Mat vis = result_frame.frame.clone();
        for (const auto_aim::ArmorPose& pose : result_frame.poses)
        {
            const std::vector<cv::Point> quad = {
                pose.armor.left_light.top,
                pose.armor.right_light.top,
                pose.armor.right_light.bottom,
                pose.armor.left_light.bottom,
            };
            cv::polylines(vis, quad, true, cv::Scalar(0, 255, 0), 2);

            //tvec/rvec 都是 3x1 的 CV_64F，把三个分量拼成两行文本标到装甲板旁
            char tvec_text[64];
            std::snprintf(tvec_text, sizeof(tvec_text), "t=[%.0f %.0f %.0f]mm",
                          pose.tvec.at<double>(0), pose.tvec.at<double>(1), pose.tvec.at<double>(2));
            char rvec_text[64];
            std::snprintf(rvec_text, sizeof(rvec_text), "r=[%.2f %.2f %.2f]",
                          pose.rvec.at<double>(0), pose.rvec.at<double>(1), pose.rvec.at<double>(2));

            //以装甲板中心为基准，rvec 在上一行、tvec 在下一行，避免重叠
            const cv::Point rvec_org(pose.armor.center.x, pose.armor.center.y - 10);
            const cv::Point tvec_org(pose.armor.center.x, pose.armor.center.y + 15);
            cv::putText(vis, rvec_text, rvec_org, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
            cv::putText(vis, tvec_text, tvec_org, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
        }

        //左上角标一行说明：配对装甲数 + 解算成功位姿数
        cv::putText(vis, "pnp (default params)  matched=" + std::to_string(result_frame.matched)
                        + " solved=" + std::to_string(result_frame.poses.size()),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比）
        auto_aim::fitToScreen(vis);

        cv::imshow("pnp", vis);

        //每满一秒打印一次取帧 FPS、检测 FPS 和当前配对/解算数
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << (capture - lastCapture) / elapsed
                      << " detect_fps=" << (detect - lastDetect) / elapsed
                      << " matched=" << result_frame.matched
                      << " solved=" << result_frame.poses.size() << '\n';
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
