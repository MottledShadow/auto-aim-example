#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "detector.hpp"
#include "hik_camera.hpp"

namespace
{

//与 detector.cpp 里 filterLightBars 一致的退化轮廓判定阈值
constexpr double kEpsilon = 1e-6;

//目标显示器分辨率：整帧标注后等比缩放到刚好放进这个尺寸
constexpr int kScreenWidth = 1920;
constexpr int kScreenHeight = 1080;

//把一个 double 转成固定小数位的短字符串，用来标注
std::string fmt(double value, int decimals)
{
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed << value;
    return oss.str();
}

//最新帧槽：只保最新一份，被覆盖写入。序号变化即代表有新数据（和相机的 LatestImagesOnly 一致）
template <typename T>
struct LatestSlot
{
    std::mutex mutex;
    std::condition_variable ready;
    T payload;
    std::uint64_t seq = 0;
    std::atomic_bool running{true};

    //生产者：覆盖 payload，序号 +1，唤醒一个消费者
    void publish(T value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            payload = std::move(value);
            ++seq;
        }
        ready.notify_one();
    }

    //消费者：等到有比 consumed 更新的数据或停止；返回 false 表示该退出了
    bool wait(std::uint64_t& consumed, T& out)
    {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock, [&] { return seq != consumed || !running; });
        if (!running)
        {
            return false;
        }
        out = payload;
        consumed = seq;
        return true;
    }

    //置停止并唤醒所有等待者
    void stop()
    {
        running = false;
        ready.notify_all();
    }
};

//处理线程交给显示线程的东西：原图 + 预处理结果（含候选轮廓）+ 生产筛选通过的灯条数，绘图交给显示线程做
struct FrameResult
{
    cv::Mat frame;
    auto_aim::PreprocessResult processed;
    std::size_t passed = 0;
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

    //两级流水线各一个最新帧槽：相机原图 → 处理产出（原图+预处理结果，绘图留给显示线程）
    LatestSlot<cv::Mat> slotRaw;
    LatestSlot<FrameResult> slotVis;

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

    //处理线程：跑 preprocess + filterLightBars（都走头文件默认参数），把原图+预处理结果+通过数发给显示线程（绘图不在这里做）
    std::thread processThread([&]
    {
        std::uint64_t consumed = 0;
        cv::Mat frame;
        while (slotRaw.wait(consumed, frame))
        {
            //二值化方法/目标色/筛选阈值都走头文件默认值，要改测灰度或红色改 detector.hpp
            auto_aim::PreprocessParams pre_params;
            auto_aim::LightBarFilterParams filter_params;

            FrameResult output;
            output.frame = frame;
            output.processed = auto_aim::preprocess(frame, pre_params);
            const std::vector<auto_aim::LightBar> bars =
                auto_aim::filterLightBars(frame, output.processed, filter_params);
            output.passed = bars.size();
            slotVis.publish(std::move(output));
            ++detectCount;
        }
    });

    //主线程负责显示+统计：HighGUI 的 imshow/waitKey 必须在主线程，标注/缩放也放这里
    std::cout << "一个窗口：逐候选标注灯条筛选四项指标（A/AR/ang/fill），绿=在范围/红=超范围；q/ESC 退出\n";
    //筛选参数默认值，主线程重算指标时逐项比对用（与处理线程用的默认值一致）
    const auto_aim::LightBarFilterParams filter_params;
    std::uint64_t consumed = 0;
    FrameResult result_frame;
    //帧率打印基准：每秒用增量算一次 FPS
    auto lastPrint = std::chrono::steady_clock::now();
    std::uint64_t lastCapture = 0;
    std::uint64_t lastDetect = 0;
    while (slotVis.wait(consumed, result_frame))
    {
        //在原图上标注：遍历全部候选，按 filterLightBars 的公式重算四项指标并逐项着色
        cv::Mat vis = result_frame.frame.clone();
        for (const auto& candidate : result_frame.processed.candidates)
        {
            const cv::RotatedRect& rect = candidate.rect;
            const double area = candidate.area;

            //取最小外接矩形的长边短边，退化轮廓跳过（与生产一致）
            const double long_side = std::max(rect.size.width, rect.size.height);
            const double short_side = std::min(rect.size.width, rect.size.height);
            if (short_side <= kEpsilon)
            {
                continue;
            }
            const double rect_area = long_side * short_side;
            if (rect_area <= kEpsilon)
            {
                continue;
            }

            //长宽比
            const double aspect_ratio = long_side / short_side;

            //中心线与竖直方向的夹角（度），方向向量模为 0 时记 90 度
            const double vx = candidate.center_line[0];
            const double vy = candidate.center_line[1];
            const double norm = std::hypot(vx, vy);
            double line_angle_deg = 90.0;
            if (norm > kEpsilon)
            {
                line_angle_deg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
            }

            //轮廓面积占外接矩形的比例
            const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

            //逐项判断是否在范围内
            const bool area_ok = area >= filter_params.min_area && area <= filter_params.max_area;
            const bool aspect_ok = aspect_ratio >= filter_params.min_aspect_ratio && aspect_ratio <= filter_params.max_aspect_ratio;
            const bool angle_ok = line_angle_deg >= filter_params.min_line_angle_deg && line_angle_deg <= filter_params.max_line_angle_deg;
            const bool fill_ok = fill_ratio >= filter_params.min_fill_ratio && fill_ratio <= filter_params.max_fill_ratio;
            const bool pass = area_ok && aspect_ok && angle_ok && fill_ok;

            //在范围绿、超范围红
            const cv::Scalar green(0, 255, 0);
            const cv::Scalar red(0, 0, 255);

            //画最小外接矩形，整体通过绿、被拒红
            cv::Point2f corners[4];
            rect.points(corners);
            for (int i = 0; i < 4; ++i)
            {
                cv::line(vis, corners[i], corners[(i + 1) % 4], pass ? green : red, 1);
            }

            //在轮廓旁边分四行标注四项值，每行按该项是否在范围单独着色
            const cv::Point anchor(cvRound(rect.center.x), cvRound(rect.center.y));
            cv::putText(vis, "A=" + fmt(area, 0), anchor + cv::Point(0, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, area_ok ? green : red, 1);
            cv::putText(vis, "AR=" + fmt(aspect_ratio, 2), anchor + cv::Point(0, 14),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, aspect_ok ? green : red, 1);
            cv::putText(vis, "ang=" + fmt(line_angle_deg, 1), anchor + cv::Point(0, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, angle_ok ? green : red, 1);
            cv::putText(vis, "fill=" + fmt(fill_ratio, 2), anchor + cv::Point(0, 42),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, fill_ok ? green : red, 1);
        }

        //左上角标一行说明：方法名 + 本帧通过灯条数
        cv::putText(vis, "lightbar filter (default params)  passed=" + std::to_string(result_frame.passed),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        //整体等比缩放到刚好放进 1920x1080（不裁切，保留完整画面便于对比）
        const double scale = std::min(
            static_cast<double>(kScreenWidth) / vis.cols,
            static_cast<double>(kScreenHeight) / vis.rows);
        if (scale < 1.0)
        {
            cv::resize(vis, vis, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        cv::imshow("lightbar filter", vis);

        //每满一秒打印一次取帧 FPS、检测 FPS 和当前通过灯条数
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - lastPrint).count();
        if (elapsed >= 1.0)
        {
            const std::uint64_t capture = captureCount.load();
            const std::uint64_t detect = detectCount.load();
            std::cout << std::fixed << std::setprecision(1)
                      << "capture_fps=" << (capture - lastCapture) / elapsed
                      << " detect_fps=" << (detect - lastDetect) / elapsed
                      << " passed=" << result_frame.passed << '\n';
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
