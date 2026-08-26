#include <chrono>
#include <iostream>
#include <thread>

#include "serial.hpp"

// 连真实 STM32 的接收冒烟测试：构造(读配置+开串口+起接收线程) → 主循环每 100ms 打印最新四元数
// 设备/波特率写死在 SerialConfig 默认值里（serial.hpp），命令行不传参
int main() {
    Serial serial;   // 构造完：串口已开、后台线程已在收

    while (true) {
        Quaternion q = serial.latest();
        std::cout << "w=" << q.w << " x=" << q.x
                  << " y=" << q.y << " z=" << q.z << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
