#include <exception>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "hik_camera.hpp"

int main()
try
{
    //构造即初始化（打开设备并启动后台采集线程），失败抛异常
    auto_aim::HikCamera camera;

    std::cout << "SPACE 拍照，q/ESC 退出\n";

    //循环取帧、预览，按空格存图
    auto_aim::HikCameraFrame frame;
    int count = 0;
    while (true)
    {
        //取最新一帧
        int result = camera.capture(frame);
        if (result != MV_OK)
        {
            std::cerr << "capture failed: 0x"
                      << std::hex << static_cast<unsigned int>(result) << '\n';
            return 2;
        }

        //在 Jetson 屏幕上预览
        cv::imshow("snapshot", frame.image);
        const int key = cv::waitKey(1);

        //空格拍照，存到 captures 目录
        if (key == ' ')
        {
            const std::string path = "captures/snapshot_" + std::to_string(count) + ".png";
            cv::imwrite(path, frame.image);
            std::cout << "saved " << path << '\n';
            ++count;
        }

        //q 或 ESC 退出
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
    }

    //关闭窗口，相机析构时自动清理
    cv::destroyAllWindows();
    return 0;
}
catch (const std::exception& ex)
{
    std::cerr << "snapshot failed: " << ex.what() << '\n';
    return 1;
}
