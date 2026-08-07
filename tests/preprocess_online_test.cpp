#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "detector.hpp"
#include "hik_camera.hpp"

namespace
{

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

//预处理产出的一对展示图
struct DisplayPair
{
    cv::Mat binary;
    cv::Mat vis;
};

//在原图上画候选轮廓、最小外接矩形、fitLine 中心线、面积数字（沿用离线测试的画法）
void drawCandidates(cv::Mat& vis, const std::vector<auto_aim::ContourCandidate>& candidates)
{
    for (const auto& candidate : candidates)
    {
        //候选轮廓（绿色）
        const std::vector<std::vector<cv::Point>> one_contour{candidate.contour};
        cv::drawContours(vis, one_contour, -1, cv::Scalar(0, 255, 0), 1);

        //最小外接矩形（黄色）
        cv::Point2f corners[4];
        candidate.rect.points(corners);
        for (int i = 0; i < 4; ++i)
        {
            cv::line(vis, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
        }

        //fitLine 中心线（红色），沿方向向量往两边延伸
        const float vx = candidate.center_line[0];
        const float vy = candidate.center_line[1];
        const float x0 = candidate.center_line[2];
        const float y0 = candidate.center_line[3];
        const float length = 30.0F;
        const cv::Point p1(cvRound(x0 - vx * length), cvRound(y0 - vy * length));
        const cv::Point p2(cvRound(x0 + vx * length), cvRound(y0 + vy * length));
        cv::line(vis, p1, p2, cv::Scalar(0, 0, 255), 1);

        //面积数字
        cv::putText(
            vis,
            "A=" + std::to_string(static_cast<int>(candidate.area)),
            candidate.rect.center,
            cv::FONT_HERSHEY_SIMPLEX,
            0.4,
            cv::Scalar(0, 0, 255),
            1);
    }
}

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

    //两级流水线各一个最新帧槽：相机原图 → 预处理产出
    LatestSlot<cv::Mat> slotRaw;
    LatestSlot<DisplayPair> slotVis;

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
        }
        //取帧结束，通知下游别再等了
        slotVis.stop();
    });

    //预处理线程：等最新原图 → 预处理 → 生成二值图和轮廓可视化图，发布到 slotVis
    std::thread preprocessThread([&]
    {
        std::uint64_t consumed = 0;
        cv::Mat frame;
        while (slotRaw.wait(consumed, frame))
        {
            const auto_aim::ArmorPreprocessResult processed = auto_aim::preprocess(frame);

            //二值图转成 3 通道，方便和另一个窗口一致
            DisplayPair pair;
            cv::cvtColor(processed.binary, pair.binary, cv::COLOR_GRAY2BGR);

            //在原图 clone 上画轮廓、最小矩形、中心线、面积
            pair.vis = frame.clone();
            drawCandidates(pair.vis, processed.candidates);

            slotVis.publish(std::move(pair));
        }
    });

    //主线程负责显示：HighGUI 的 imshow/waitKey 必须在主线程
    std::cout << "两个窗口：binary 二值化，contours 轮廓+最小矩形；q/ESC 退出\n";
    std::uint64_t consumed = 0;
    DisplayPair pair;
    while (slotVis.wait(consumed, pair))
    {
        cv::imshow("binary", pair.binary);
        cv::imshow("contours", pair.vis);
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
