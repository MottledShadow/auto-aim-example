#pragma once

#include <string>
#include <thread>

#include "latest_slot.hpp"

// 串口连接参数
struct SerialConfig {
    std::string device = "/dev/ttyTHS1";
    int baudrate = 460800;
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

// 发给 STM32 的目标：检测标志 + 相机系坐标 x/y/z（单位由调用方约定，这里只负责搬运）
struct TargetOutput {
    bool detected = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

class Serial {
public:
    explicit Serial();
    ~Serial();

    // 取最近一帧四元数（线程安全，供主线程调用）
    Quaternion latest();

    // 发一帧目标（16 字节：0xA5 + 检测标志 + x/y/z + CRC16），主线程调用
    void send(const TargetOutput& target);

private:
    // 后台线程体：按 0x5A 帧头同步 → 读定长负载 → 解析四元数 → 存最新值
    void receiveLoop();

    SerialConfig config_;
    int fd_ = -1;
    std::thread thread_;

    // 最新四元数槽：后台线程 publish、主线程 latest() 取（Serial 是全局命名空间，显式限定 auto_aim::）
    auto_aim::LatestSlot<Quaternion> slot_;
};
