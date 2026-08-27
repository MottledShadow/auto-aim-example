#include <cfloat>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "coordinate_transform.hpp"
#include "detector.hpp"
#include "hik_camera.hpp"
#include "latest_slot.hpp"
#include "serial.hpp"

namespace
{

// 按 PnP 点序 left.top → right.top → right.bottom → left.bottom 连四条边画装甲框
void drawArmorQuad(cv::Mat& vis, const auto_aim::detector::Armor& armor, const cv::Scalar& color, int thickness)
{
    const cv::Point2f pts[4] = {
        armor.leftLight.top,
        armor.rightLight.top,
        armor.rightLight.bottom,
        armor.leftLight.bottom,
    };
    for (int i = 0; i < 4; ++i)
    {
        cv::line(vis, pts[i], pts[(i + 1) % 4], color, thickness);
    }
    cv::circle(vis, armor.center, 4, color, -1);
}

int run()
{
    // 1. 构造三件套（RAII，失败抛异常）：相机 + 串口(后台在收四元数) + 坐标变换
    auto_aim::hik_camera::HikCamera camera;
    auto_aim::serial::Serial serial;
    auto_aim::tracker::CoordinateTransform transform;   // 构造时自读手眼标定(config/hand_eye_calibration.yml)
    if (!transform.error().empty())
    {
        std::cerr << "warning: 未加载手眼标定，回退到几何重映射: " << transform.error() << '\n';
    }

    // 2. 共享图槽：识别线程取帧时把这帧图存进来，主线程画图时取最新
    auto_aim::LatestSlot<cv::Mat> frameSlot;

    // 3. 帧源回调：相机取帧 → 存图 → 填 FrameInput(图 + 硬件时间戳 + 当下四元数)，喂给识别线程
    auto_aim::detector::Detector detector([&](auto_aim::detector::FrameInput& input) {
        auto_aim::hik_camera::HikCameraFrame frame;
        if (camera.capture(frame) != MV_OK)
        {
            return false;   // 超时/出错，识别线程 continue
        }
        frameSlot.publish(frame.image);
        const auto_aim::serial::Quaternion q = serial.latest();
        input.image = frame.image;
        if (!frame.timestampNs.has_value())
        {
            return false;
        }
        input.timestampNs = *frame.timestampNs;
        input.quaternion = cv::Vec4d(q.w, q.x, q.y, q.z);
        return true;
    });   // 构造即起后台识别线程

    // 4. 预览窗口（Jetson 屏幕），可缩放保持宽高比
    cv::namedWindow("full_chain", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("full_chain", 1280, 800);

    std::cout << "full chain running: ESC quit\n";

    // 5. 主循环：取最新结果+最新图 → 坐标变换 → 选目标 → 串口发 → 画面显示
    while (true)
    {
        // 5.0 取识别线程发布的最新结果 + 与之同源的最新图（相差至多一帧，冒烟测试足够）
        auto_aim::detector::DetectionResult det = detector.latest();
        cv::Mat display = frameSlot.latest();
        if (display.empty())
        {
            continue;   // 还没收到第一帧
        }
        display = display.clone();

        // 5a. 坐标变换：相机系装甲板 → 世界系(FLU, mm)，world 与 det.armors 同序
        std::vector<auto_aim::tracker::TrackedArmor> world = transform.toWorld(det.armors, det.quaternion);

        // 5b. 选目标：离画面中心最近 = distanceToPrincipalPoint 最小的那块
        int best = -1;
        float bestDist = FLT_MAX;
        for (std::size_t i = 0; i < det.armors.size(); ++i)
        {
            if (det.armors[i].distanceToPrincipalPoint < bestDist)
            {
                bestDist = det.armors[i].distanceToPrincipalPoint;
                best = static_cast<int>(i);
            }
        }

        // 5c. 组目标帧并发送：检到就发选中块的世界系坐标，没检到发 detected=false
        auto_aim::serial::TargetOutput target;
        if (best >= 0)
        {
            target.detected = true;
            target.x = static_cast<float>(world[best].position[0]);
            target.y = static_cast<float>(world[best].position[1]);
            target.z = static_cast<float>(world[best].position[2]);
        }
        serial.send(target);

        // 5d. 画所有装甲板：未选中绿色细线，选中块黄色粗线
        for (std::size_t i = 0; i < det.armors.size(); ++i)
        {
            const bool chosen = (static_cast<int>(i) == best);
            const cv::Scalar color = chosen ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
            drawArmorQuad(display, det.armors[i], color, chosen ? 3 : 2);

            // 数字 + 置信度（number 可能为空，显示 ?）
            const std::string label =
                (det.armors[i].number.empty() ? "?" : det.armors[i].number) +
                cv::format(" %.2f", det.armors[i].confidence);
            cv::putText(display, label, det.armors[i].center + cv::Point2f(6, -6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

            // 选中块额外标世界系坐标(mm)
            if (chosen)
            {
                const std::string coord = cv::format("x=%.0f y=%.0f z=%.0f mm",
                                                      world[i].position[0], world[i].position[1], world[i].position[2]);
                cv::putText(display, coord, det.armors[i].center + cv::Point2f(6, 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            }
        }

        // 左上角状态叠字：发送标志 + 发送坐标 + 实时四元数
        char sendText[96];
        std::snprintf(sendText, sizeof(sendText), "send detected=%d x=%.0f y=%.0f z=%.0f",
                      target.detected ? 1 : 0, target.x, target.y, target.z);
        cv::putText(display, sendText, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        char quatText[96];
        std::snprintf(quatText, sizeof(quatText), "quat w=%.3f x=%.3f y=%.3f z=%.3f",
                      det.quaternion[0], det.quaternion[1], det.quaternion[2], det.quaternion[3]);
        cv::putText(display, quatText, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

        cv::imshow("full_chain", display);
        if (cv::waitKey(1) == 27)   // ESC
        {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}

}  // namespace

int main()
try
{
    return run();
}
catch (const std::exception& ex)
{
    std::cerr << "error: " << ex.what() << '\n';
    return 1;
}
