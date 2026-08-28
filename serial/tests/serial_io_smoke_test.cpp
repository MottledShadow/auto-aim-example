#include <chrono>
#include <iostream>
#include <thread>

#include "serial.hpp"

// 连真实 STM32 的收发冒烟测试：构造(读配置+开串口+起接收线程) → 主循环每 100ms 打印最新四元数、并回发一帧目标
// 设备/波特率写死在 SerialConfig 默认值里（serial.hpp），命令行不传参
int main() {
    auto_aim::serial::Serial serial;   // 构造完：串口已开、后台线程已在收

    while (true) {
        // 收：打印最新四元数
        auto_aim::serial::Quaternion q = serial.latest();
        std::cout << "recv w=" << q.w << " x=" << q.x
                  << " y=" << q.y << " z=" << q.z << '\n';

        // 发：回发一帧固定测试目标，验证 STM32 侧能按 0xA5/16 字节帧解出
        auto_aim::serial::TargetOutput target;
        target.detected = true;
        target.x = 1.0f;
        target.y = 2.0f;
        target.z = 3.0f;
        serial.send(target);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
