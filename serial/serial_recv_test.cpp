#include <chrono>
#include <iostream>
#include <thread>

#include "serial.hpp"

// 连真实 STM32 的接收冒烟测试：构造(读配置+起接收线程) → 主循环每 100ms 打印最新四元数
// 配置路径写死在 Serial 里，命令行不再传
int main() {
    Serial serial;   // 构造完：配置已读、串口已开、线程已在收

    while (true) {
        Quaternion q = serial.latest();

        std::cout << "w=" << q.w
                  << " x=" << q.x
                  << " y=" << q.y
                  << " z=" << q.z << '\n';

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
