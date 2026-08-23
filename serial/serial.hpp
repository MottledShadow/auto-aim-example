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

// STM32 发来的四元数（分量顺序 w/x/y/z，这里只负责搬运，不做换算）
struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// 串口收：构造时读 yml → 打开并配置串口 → 起后台线程持续收 STM32 的四元数帧
// 帧格式：帧头 0x5A + 4×float32(小端) = w/x/y/z，无 CRC，定长 17 字节
// 配置路径写死在 serial.cpp（serial/config/serial_config.yml），要改参数直接改 yml
class Serial {
public:
    explicit Serial();
    ~Serial();

    // 取最近一帧四元数（线程安全，供主线程调用）
    Quaternion latest();

private:
    // 后台线程体：按 0x5A 帧头同步 → 读定长负载 → 解析四元数 → 存最新值
    void receiveLoop();

    SerialConfig config_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex mutex_;
    Quaternion latest_;
};
