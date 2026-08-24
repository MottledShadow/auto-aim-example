#pragma once

#include <cstddef>
#include <cstdint>

// CRC-16/CCITT 校验（poly 0x1021，reflected 0x8408，小端输出）
// RM 裁判系统串口协议用的那套；纯查表算法，无硬件依赖，可跨平台复用。
namespace crc {

// CRC16 初始值
inline constexpr std::uint16_t kInitValue = 0xFFFF;

// 计算 CRC16 校验值（小端，低字节在前）
std::uint16_t calc(const std::uint8_t* data, std::size_t length, std::uint16_t init = kInitValue);

// 校验帧末尾 2 字节（小端）是否等于前面数据的 CRC16
bool verify(const std::uint8_t* data, std::size_t length);

// 计算 CRC16 并写入帧末尾预留的最后 2 字节（小端，低字节在前）
void append(std::uint8_t* data, std::size_t length);

}  // namespace crc
