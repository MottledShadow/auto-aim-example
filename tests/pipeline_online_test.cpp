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

//处理线程交给显示线程的东西：原图 + 该阶段需要的中间产物，绘图交给显示线程做
struct FrameResult
{
    cv::Mat frame;
    //preprocess / lightbar 共用：单次预处理结果（含二值图和候选轮廓）
    auto_aim::PreprocessResult processed;
    //armor 用：筛选通过的灯条
    std::vector<auto_aim::LightBar> bars;
    //pnp 用：解算出的位姿
    std::vector<auto_aim::ArmorPose> poses;
    //本帧计数（含义随阶段而变：通过灯条数 / 配对装甲数）
    std::size_t count = 0;
};

int run(const std::string& stage)
{
    //只有 pnp 阶段需要相机标定，失败只告警不退出（solvePnp 会返回空位姿，窗口只画不出位姿）
    auto_aim::CameraCalibration calibration;
    if (stage == "pnp")
    {
        calibration = auto_aim::loadCalibration();
        if (!calibration.error.empty())
        {
            std::cerr << "calibration load failed: " << calibration.error << '\n';
        }
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

    //两级流水线各一个最新帧槽：相机原图 → 处理产出（原图+中间产物，绘图留给显示线程）
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

    //处理线程：按 stage 跑到对应阶段（都走头文件默认参数），把原图+中间产物发给显示线程（绘图不在这里做）
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

            FrameResult output;
            output.frame = frame;

            if (stage == "preprocess")
            {
                //灰度二值化跑一次，供显示线程展示二值图
                output.processed = auto_aim::preprocess(frame, pre_params);
            }
            else if (stage == "lightbar")
            {
                //只跑到筛选，候选留给显示线程逐项标注，通过灯条数用于自检
                output.processed = auto_aim::preprocess(frame, pre_params);
                const std::vector<auto_aim::LightBar> bars =
                    auto_aim::filterLightBars(frame, output.processed, filter_params);
                output.count = bars.size();
            }
            else if (stage == "armor")
            {
                //跑到配对，灯条留给显示线程重算配对指标画框
                const auto_aim::PreprocessResult processed = auto_aim::preprocess(frame, pre_params);
                output.bars = auto_aim::filterLightBars(frame, processed, filter_params);
                const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(output.bars, matcher_params);
                output.count = armors.size();
            }
            else
            {
                //pnp：跑到位姿解算，位姿留给显示线程画框标注
                const auto_aim::PreprocessResult processed = auto_aim::preprocess(frame, pre_params);
                const std::vector<auto_aim::LightBar> bars =
                    auto_aim::filterLightBars(frame, processed, filter_params);
                const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);
                output.poses = auto_aim::solvePnp(armors, calibration, pnp_params);
                output.count = armors.size();
            }

            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //主线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，标注/缩放也放这里
    //窗口标题与提示按阶段区分
    std::string window_name;
    if (stage == "preprocess")
    {
        window_name = "binarize";
        std::cout << "一个窗口：灰度阈值二值化；q/ESC 退出\n";
    }
    else if (stage == "lightbar")
    {
        window_name = "lightbar filter";
        std::cout << "一个窗口：逐候选标注灯条筛选四项指标（A/AR/ang/fill），绿=在范围/红=超范围；q/ESC 退出\n";
    }
    else if (stage == "armor")
    {
        window_name = "armor match";
        std::cout << "一个窗口：对所有同色灯条对按配对公式画装甲框(接受绿/被拒红)并逐项标注；q/ESC 退出\n";
    }
    else
    {
        window_name = "pnp";
        std::cout << "一个窗口：画装甲框并标注每块装甲的 rvec/tvec；q/ESC 退出\n";
    }

    //筛选/配对参数默认值，主线程重算指标画框时用（与处理线程用的默认值一致）
    const auto_aim::LightBarFilterParams filter_params;
    const auto_aim::LightBarMatcherParams matcher_params;
    std::uint64_t consumed = 0;
    FrameResult result_frame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    while (slotVis.wait(consumed, result_frame))
    {
        //按阶段标注，得到一张待显示图 vis；打印用的额外后缀随阶段而变
        cv::Mat vis;
        std::string fps_suffix;

        if (stage == "preprocess")
        {
            //灰度二值图转 BGR 方便加标注
            cv::cvtColor(result_frame.processed.binary, vis, cv::COLOR_GRAY2BGR);
            cv::putText(vis, "gray thresh", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
        else if (stage == "lightbar")
        {
            //遍历全部候选，按 filterLightBars 的公式重算四项指标并逐项着色
            vis = result_frame.frame.clone();
            auto_aim::drawLightBarMetrics(vis, result_frame.processed.candidates, filter_params);
            cv::putText(vis, "lightbar filter (default params)  passed=" + std::to_string(result_frame.count),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            fps_suffix = " passed=" + std::to_string(result_frame.count);
        }
        else if (stage == "armor")
        {
            //遍历全部同色灯条对，按 matchArmors 的公式重算配对指标并画框+逐项着色
            vis = result_frame.frame.clone();
            const std::size_t draw_passed = auto_aim::drawArmorMetrics(vis, result_frame.bars, matcher_params);
            cv::putText(vis, "armor match (default params)  matched=" + std::to_string(result_frame.count)
                            + " draw_passed=" + std::to_string(draw_passed),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            fps_suffix = " matched=" + std::to_string(result_frame.count);
        }
        else
        {
            //pnp：逐位姿画装甲四边形并标注 rvec/tvec
            vis = result_frame.frame.clone();
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
            cv::putText(vis, "pnp (default params)  matched=" + std::to_string(result_frame.count)
                            + " solved=" + std::to_string(result_frame.poses.size()),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            fps_suffix = " matched=" + std::to_string(result_frame.count)
                       + " solved=" + std::to_string(result_frame.poses.size());
        }

        //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比）
        auto_aim::fitToScreen(vis);
        cv::imshow(window_name, vis);

        //每满一秒打印一次取帧 FPS、检测 FPS 和阶段计数
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << (capture - lastCapture) / elapsed
                      << " detect_fps=" << (detect - lastDetect) / elapsed
                      << fps_suffix << '\n';
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

int main(int argc, char** argv)
{
    //第一个参数选调试阶段：preprocess | lightbar | armor | pnp，逐阶段实时预览
    const std::string stage = (argc > 1) ? argv[1] : "";
    if (stage != "preprocess" && stage != "lightbar" && stage != "armor" && stage != "pnp")
    {
        std::cerr << "usage: pipeline_online_test <preprocess|lightbar|armor|pnp>\n";
        return 1;
    }

    try
    {
        return run(stage);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test failed: " << exception.what() << '\n';
        return 3;
    }
}
