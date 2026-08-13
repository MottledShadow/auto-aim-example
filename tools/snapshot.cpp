#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "hik_camera.hpp"

int main()
{
    //初始化相机（打开设备并启动后台采集线程）
    auto_aim::HikCamera camera;
    int result = camera.initialize();
    if (result != MV_OK)
    {
        std::cerr << "initialize failed: 0x"
                  << std::hex << static_cast<unsigned int>(result) << '\n';
        return 1;
    }

    std::cout << "SPACE 拍照，q/ESC 退出\n";

    //循环取帧、预览，按空格存图
    auto_aim::HikCameraFrame frame;
    int count = 0;
    while (true)
    {
        //取最新一帧
        result = camera.capture(frame);
        if (result != MV_OK)
        {
            std::cerr << "capture failed: 0x"
                      << std::hex << static_cast<unsigned int>(result) << '\n';
            camera.shutdown();
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

    //关闭窗口与相机
    cv::destroyAllWindows();
    camera.shutdown();
    return 0;
}
