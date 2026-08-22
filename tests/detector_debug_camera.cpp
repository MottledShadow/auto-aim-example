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
#include "lightbar_detector.hpp"
#include "hik_camera.hpp"
#include "latest_slot.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

namespace
{

//单块装甲板的数字诊断标注（与去留过滤解耦：keep 只决定颜色，不影响 PnP）
struct NumberAnno
{
    cv::Point2f center;
    std::vector<cv::Point> quad;
    std::string number;
    float confidence = 0.0f;
    bool keep = false;
};

//处理线程每帧跑完整流水线，把原图 + 所有中间产物一并塞进来交给显示线程逐层叠画
struct FrameResult
{
    cv::Mat frame;
    std::uint64_t timestamp = 0;   // 硬件时间戳(设备 tick)，从取帧线程一路带下来
    //preprocess/lightbar 用：单次预处理结果（含二值图和候选轮廓）
    auto_aim::PreprocessResult processed;
    //armor 用：筛选通过的灯条
    std::vector<auto_aim::LightBar> bars;
    //number 用：每块配对装甲的数字诊断标注（不过滤）
    std::vector<NumberAnno> numbers;
    //pnp 用：对全部配对装甲解算出位姿的装甲板（绕开分类过滤，Armor 自带 rvec/tvec）
    std::vector<auto_aim::Armor> solved;
    //各阶段计数，画在状态栏
    std::size_t barCount = 0;
    std::size_t armorCount = 0;
    std::size_t kept = 0;
};

int run()
{
    //几何检测器无状态，参数走头文件默认值；要改阈值改 lightbar_detector.hpp
    auto_aim::LightbarDetector detector;

    //数字分类器：构造时加载一次网络+标签（有状态、开销大），失败只告警（不挡 PnP）
    auto_aim::NumberClassifierParams numberParams;
    auto_aim::NumberClassifier classifier(numberParams);
    if (!classifier.error().empty())
    {
        std::cerr << "classifier load failed: " << classifier.error() << '\n';
    }

    //PnP 求解器：构造时自己读标定（路径写死），失败只告警（solve 会返回空位姿）
    const auto_aim::PnpSolver pnpSolver;
    if (!pnpSolver.error().empty())
    {
        std::cerr << "calibration load failed: " << pnpSolver.error() << '\n';
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

    //两级流水线各一个最新帧槽：相机原图 → 处理产出（原图+全部中间产物，绘图留给显示线程）
    auto_aim::LatestSlot<auto_aim::FrameInput> slotRaw;
    auto_aim::LatestSlot<FrameResult> slotVis;

    //帧率计数：取帧线程累加 captureCount，处理线程累加 detectCount，显示线程按秒读差值
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
            //打包成取帧对象：图像 + 硬件时间戳(设备 tick)，四元数走默认单位值(IMU 未接入)
            auto_aim::FrameInput input;
            input.image = frame.image.clone();
            input.timestamp = frame.hardwareTimestamp;
            slotRaw.publish(std::move(input));
            ++captureCount;
        }
        //取帧结束，通知下游别再等了
        slotVis.stop();
    });

    //处理线程：每帧跑完整流水线（都走头文件默认参数），把原图+全部中间产物发给显示线程（绘图不在这里做）
    std::thread processThread([&]
    {
        std::uint64_t consumed = 0;
        auto_aim::FrameInput raw;
        while (slotRaw.wait(consumed, raw))
        {
            const cv::Mat& frame = raw.image;
            FrameResult output;
            output.frame = frame;
            output.timestamp = raw.timestamp;

            //几何三阶段：预处理 → 灯条筛选 → 装甲配对（armors 是 PnP 的输入，与分类无关）
            output.processed = detector.preprocess(frame);
            output.bars = detector.filterLightBars(frame, output.processed);
            const std::vector<auto_aim::Armor> armors = detector.matchArmors(output.bars);
            output.barCount = output.bars.size();
            output.armorCount = armors.size();

            //逐块 diagnose 拿数字/置信度（不过滤），按四条去留规则算 KEEP/REJECT 只用于着色
            for (const auto_aim::Armor& armor : armors)
            {
                const auto_aim::NumberClassifier::Diagnosis d = classifier.diagnose(frame, armor);
                NumberAnno anno;
                anno.center = armor.center;
                anno.quad = {
                    armor.leftLight.top,
                    armor.rightLight.top,
                    armor.rightLight.bottom,
                    armor.leftLight.bottom,
                };
                anno.number = classifier.labels()[d.classIndex];
                anno.confidence = d.confidence;

                bool keep = true;
                if (d.confidence < numberParams.confidenceThreshold)
                {
                    keep = false;
                }
                else if (anno.number != "1" && anno.number != "3" && anno.number != "guard")
                {
                    keep = false;
                }
                else if (armor.type == auto_aim::ArmorType::Large && anno.number != "1")
                {
                    keep = false;
                }
                else if (armor.type == auto_aim::ArmorType::Small && anno.number == "1")
                {
                    keep = false;
                }
                anno.keep = keep;
                if (keep)
                {
                    ++output.kept;
                }
                output.numbers.push_back(anno);
            }

            //PnP 直接对全部配对装甲解算（绕开分类过滤，没贴贴纸也能看到位姿）
            output.solved = pnpSolver.solve(armors);

            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //显示线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，标注/缩放也放这里
    const std::string windowName = "detector debug (camera)";
    std::cout << "单窗口按键切换/叠加各阶段标注层：\n"
                 "  1=预处理(二值背景+候选轮廓)  2=灯条层  3=配对层  4=数字标注  5=PnP 标注\n"
                 "  可多层叠加；q/Q/ESC 退出。默认只开配对层 3\n";

    //各层开关（下标 1..5 对应 5 个阶段），默认只开配对层 3
    bool layer[6] = {false, false, false, true, false, false};

    std::uint64_t consumed = 0;
    FrameResult resultFrame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    double captureFps = 0.0;
    double detectFps = 0.0;
    while (slotVis.wait(consumed, resultFrame))
    {
        //层 1 开：背景换成二值图并画候选轮廓；否则背景用原图
        cv::Mat vis;
        if (layer[1])
        {
            cv::cvtColor(resultFrame.processed.binary, vis, cv::COLOR_GRAY2BGR);
            auto_aim::drawCandidates(vis, resultFrame.processed.candidates);
        }
        else
        {
            vis = resultFrame.frame.clone();
        }

        //层 2：灯条层（逐候选按公式重算四项指标，各一色细线 + 左上角图例）
        if (layer[2])
        {
            auto_aim::drawLightBarMetrics(vis, resultFrame.processed.candidates, detector.filterParams);
        }

        //层 3：配对层（灯条标索引 + 按公式画装甲框 + 左上角图例 i+j 指标）
        if (layer[3])
        {
            auto_aim::drawArmorMetrics(vis, resultFrame.bars, detector.matcherParams);
        }

        //层 4：数字标注（逐块旁标 数字 置信度，KEEP 绿/REJECT 红，无论过没过筛选都标）
        if (layer[4])
        {
            for (const NumberAnno& anno : resultFrame.numbers)
            {
                const cv::Scalar color = anno.keep ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                cv::polylines(vis, anno.quad, true, color, 2);
                char label[32];
                std::snprintf(label, sizeof(label), "%s %.2f", anno.number.c_str(), anno.confidence);
                cv::putText(vis, label, cv::Point(anno.center.x, anno.center.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }

        //层 5：PnP 标注（逐块画框 + 两行 rvec/tvec）
        if (layer[5])
        {
            for (const auto_aim::Armor& armor : resultFrame.solved)
            {
                const std::vector<cv::Point> quad = {
                    armor.leftLight.top,
                    armor.rightLight.top,
                    armor.rightLight.bottom,
                    armor.leftLight.bottom,
                };
                cv::polylines(vis, quad, true, cv::Scalar(0, 255, 0), 2);

                //tvec/rvec 都是 3x1 的 CV_64F，把三个分量拼成两行文本标到装甲板旁
                char tvecText[64];
                std::snprintf(tvecText, sizeof(tvecText), "t=[%.0f %.0f %.0f]mm",
                              armor.tvec.at<double>(0), armor.tvec.at<double>(1), armor.tvec.at<double>(2));
                char rvecText[64];
                std::snprintf(rvecText, sizeof(rvecText), "r=[%.2f %.2f %.2f]",
                              armor.rvec.at<double>(0), armor.rvec.at<double>(1), armor.rvec.at<double>(2));

                const cv::Point rvecOrg(armor.center.x, armor.center.y - 10);
                const cv::Point tvecOrg(armor.center.x, armor.center.y + 15);
                cv::putText(vis, rvecText, rvecOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 0), 2);
                cv::putText(vis, tvecText, tvecOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 0), 2);
            }
        }

        //底部状态栏：当前开启的层 + 各阶段计数 + capture/detect FPS（放底部避开左上角图例）
        std::string layersText = "layers[1-5]:";
        for (int i = 1; i <= 5; ++i)
        {
            layersText += layer[i] ? static_cast<char>('0' + i) : '-';
        }
        char status[160];
        std::snprintf(status, sizeof(status),
                      "%s  bars=%zu armors=%zu kept=%zu solved=%zu  cap=%.1f det=%.1f",
                      layersText.c_str(), resultFrame.barCount, resultFrame.armorCount,
                      resultFrame.kept, resultFrame.solved.size(), captureFps, detectFps);
        cv::putText(vis, status, cv::Point(10, vis.rows - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

        //整体等比缩放到刚好放进屏幕（不裁切，保留完整画面便于对比）
        auto_aim::fitToScreen(vis);
        cv::imshow(windowName, vis);

        //每满一秒更新一次取帧/检测 FPS（供状态栏显示，同时打印一行）
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            captureFps = (capture - lastCapture) / elapsed;
            detectFps = (detect - lastDetect) / elapsed;
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << captureFps
                      << " detect_fps=" << detectFps
                      << " bars=" << resultFrame.barCount
                      << " armors=" << resultFrame.armorCount
                      << " kept=" << resultFrame.kept
                      << " solved=" << resultFrame.solved.size();
            std::cout << '\n';
            lastPrint = now;
            lastCapture = capture;
            lastDetect = detect;
        }

        //按 1..5 切换对应层开关，q/Q/ESC 退出
        const int key = cv::waitKey(1);
        if (key >= '1' && key <= '5')
        {
            layer[key - '0'] = !layer[key - '0'];
        }
        else if (key == 'q' || key == 'Q' || key == 27)
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
