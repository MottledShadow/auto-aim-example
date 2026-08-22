#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

// 串口连接参数（从 serial_config.yml 读）
struct SerialConfig {
    std::string device = "/dev/ttyTHS1";
    int baudrate = 115200;
    int data_bits = 8;                  // 5 / 6 / 7 / 8
    std::string parity = "none";        // none / even / odd
    int stop_bits = 1;                  // 1 / 2
    std::string flow_control = "none";  // none / hardware(RTS/CTS)
};

// STM32 发来的欧拉角（单位按 STM32 约定，这里只负责搬运，不做换算）
struct EulerAngles {
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
};

// 串口收：构造时读 yml → 打开并配置串口 → 起后台线程持续收 STM32 的欧拉角帧
// 帧格式：帧头 0x5A + 3×float32(小端) = yaw/pitch/roll，无 CRC，定长 13 字节
// 配置路径写死在 serial.cpp（serial/config/serial_config.yml），要改参数直接改 yml
class Serial {
public:
    explicit Serial();
    ~Serial();

    // 取最近一帧欧拉角（线程安全，供主线程调用）
    EulerAngles latest();

private:
    // 后台线程体：按 0x5A 帧头同步 → 读定长负载 → 解析欧拉角 → 存最新值
    void receiveLoop();

    SerialConfig config_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex mutex_;
    EulerAngles latest_;
};
