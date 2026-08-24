#include "serial.hpp"

#include "crc.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/core.hpp>

// 帧头 0x5A，之后 4 个 float32 小端 = w/x/y/z，末尾 2 字节小端 CRC16
static const unsigned char kFrameHeader = 0x5A;
static const int kPayloadSize = 16;   // 4 × float32
static const int kCrcSize = 2;        // CRC16，小端 2 字节
static const int kFrameSize = 1 + kPayloadSize + kCrcSize;   // 帧头 + 负载 + CRC = 19

// 把 yml 里的整数波特率映射成 termios 的 Bxxxx 常量
static speed_t toBaudConstant(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
    }
}

// 用 cv::FileStorage 读串口配置
static SerialConfig loadSerialConfig() {
    const std::string path = "serial/config/serial_config.yml";
    SerialConfig config;

    // 打开 OpenCV YAML，读不到就抛异常
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot open serial config: " + path);
    }

    // 逐项读，缺哪项就沿用结构体默认值
    if (!storage["device"].empty()) {
        storage["device"] >> config.device;
    }
    if (!storage["baudrate"].empty()) {
        storage["baudrate"] >> config.baudrate;
    }
    if (!storage["data_bits"].empty()) {
        storage["data_bits"] >> config.data_bits;
    }
    if (!storage["parity"].empty()) {
        storage["parity"] >> config.parity;
    }
    if (!storage["stop_bits"].empty()) {
        storage["stop_bits"] >> config.stop_bits;
    }
    if (!storage["flow_control"].empty()) {
        storage["flow_control"] >> config.flow_control;
    }

    return config;
}

// 初始化列表里直接用返回值构造 config_
Serial::Serial() : config_(loadSerialConfig()) {
    // 1. 打开串口
    fd_ = open(config_.device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        throw std::runtime_error("open " + config_.device + " failed: " + std::strerror(errno));
    }

    // 2. 读当前串口配置
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));
    }

    // 3. 波特率
    speed_t speed = toBaudConstant(config_.baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 4. raw 模式
    cfmakeraw(&tty);

    // 5. 数据位
    tty.c_cflag &= ~CSIZE;
    switch (config_.data_bits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            throw std::runtime_error("unsupported data_bits: " + std::to_string(config_.data_bits));
    }

    // 6. 校验位
    if (config_.parity == "none") {
        tty.c_cflag &= ~PARENB;
    } else if (config_.parity == "even") {
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
    } else if (config_.parity == "odd") {
        tty.c_cflag |= PARENB;
        tty.c_cflag |= PARODD;
    } else {
        throw std::runtime_error("unsupported parity: " + config_.parity);
    }

    // 7. 停止位
    if (config_.stop_bits == 1) {
        tty.c_cflag &= ~CSTOPB;
    } else if (config_.stop_bits == 2) {
        tty.c_cflag |= CSTOPB;
    } else {
        throw std::runtime_error("unsupported stop_bits: " + std::to_string(config_.stop_bits));
    }

    // 8. 流控
    if (config_.flow_control == "none") {
        tty.c_cflag &= ~CRTSCTS;
    } else if (config_.flow_control == "hardware") {
        tty.c_cflag |= CRTSCTS;
    } else {
        throw std::runtime_error("unsupported flow_control: " + config_.flow_control);
    }

    // 允许接收
    tty.c_cflag |= CREAD | CLOCAL;

    // read() 最多等 0.1s 就返回，这样析构时线程能及时退出、被 join
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    // 9. 写回串口
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
    }

    std::cout << "UART opened: " << config_.device << " @ " << config_.baudrate << '\n';

    // 10. 起后台接收线程
    running_ = true;
    thread_ = std::thread(&Serial::receiveLoop, this);
}

Serial::~Serial() {
    // 停线程 → 等它退出 → 关串口
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (fd_ >= 0) {
        close(fd_);
    }
}

Quaternion Serial::latest() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

void Serial::receiveLoop() {
    // 整帧缓冲：frame[0] 帧头，frame[1..16] 负载，frame[17..18] CRC
    std::uint8_t frame[kFrameSize];

    while (running_) {
        // 1. 逐字节找帧头 0x5A
        std::uint8_t byte = 0;
        ssize_t n = read(fd_, &byte, 1);
        if (n <= 0) {
            continue;   // 超时没数据 / 读失败，重来
        }
        if (byte != kFrameHeader) {
            continue;
        }
        frame[0] = byte;

        // 2. 帧头对上，读满 负载 + CRC 共 18 字节到 frame + 1
        int got = 0;
        const int rest = kPayloadSize + kCrcSize;
        while (got < rest && running_) {
            ssize_t m = read(fd_, frame + 1 + got, rest - got);
            if (m <= 0) {
                continue;
            }
            got += m;
        }
        if (got < rest) {
            break;   // running_ 被置 false，退出线程
        }

        // 3. CRC16 校验整帧（帧头+负载），错帧丢弃、回去重新找帧头
        if (!crc::verify(frame, kFrameSize)) {
            continue;
        }

        // 4. 小端 float32 直接 memcpy 成 w/x/y/z（负载从 frame + 1 开始）
        Quaternion q;
        std::memcpy(&q.w, frame + 1 + 0, 4);
        std::memcpy(&q.x, frame + 1 + 4, 4);
        std::memcpy(&q.y, frame + 1 + 8, 4);
        std::memcpy(&q.z, frame + 1 + 12, 4);

        // 5. 存最新值，供主线程 latest() 取
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = q;
    }
}
