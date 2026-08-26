#include "crc.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

// 本模块 CRC = CRC-16/MCRF4XX（poly 0x1021、reflected 0x8408、init 0xFFFF、无 xorout）。
// 帧格式见 serial.cpp：帧头 0x5A + 16 字节负载（4×float32）+ 2 字节小端 CRC = 19 字节。

// 1. 已知答案测试：标准校验向量 "123456789" → 0x6F91（独立参照，抓算法/查表回归）
TEST(CrcCalc, KnownAnswerVector) {
    const std::uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc::calc(data, sizeof(data)), 0x6F91);
}

// 2. append 后 verify 自洽：写进末尾 2 字节，再校验应通过
TEST(CrcRoundTrip, AppendThenVerify) {
    // 前 5 字节任意负载，末尾 2 字节留给 CRC
    std::vector<std::uint8_t> frame = {0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00};
    crc::append(frame.data(), frame.size());
    EXPECT_TRUE(crc::verify(frame.data(), frame.size()));
}

// 3. append 落位是小端：末尾两字节 == calc 结果的低字节、高字节
TEST(CrcRoundTrip, AppendIsLittleEndian) {
    std::vector<std::uint8_t> frame = {0xAA, 0xBB, 0xCC, 0x00, 0x00};
    const std::uint16_t expected = crc::calc(frame.data(), frame.size() - 2);
    crc::append(frame.data(), frame.size());
    EXPECT_EQ(frame[frame.size() - 2], static_cast<std::uint8_t>(expected & 0xFF));
    EXPECT_EQ(frame[frame.size() - 1], static_cast<std::uint8_t>(expected >> 8));
}

// 4. 篡改任一负载字节 → verify 失败
TEST(CrcVerify, TamperedPayloadFails) {
    std::vector<std::uint8_t> frame = {0x10, 0x20, 0x30, 0x40, 0x00, 0x00};
    crc::append(frame.data(), frame.size());
    frame[1] ^= 0xFF;   // 翻掉一个负载字节
    EXPECT_FALSE(crc::verify(frame.data(), frame.size()));
}

// 5. 篡改 CRC 字节本身 → verify 失败
TEST(CrcVerify, TamperedCrcFails) {
    std::vector<std::uint8_t> frame = {0x10, 0x20, 0x30, 0x40, 0x00, 0x00};
    crc::append(frame.data(), frame.size());
    frame[frame.size() - 1] ^= 0xFF;   // 翻掉 CRC 高字节
    EXPECT_FALSE(crc::verify(frame.data(), frame.size()));
}

// 6. 退化输入：nullptr / 长度 <= 2 → verify 返回 false，不崩
TEST(CrcVerify, DegenerateInputRejected) {
    EXPECT_FALSE(crc::verify(nullptr, 19));
    const std::uint8_t two[] = {0x12, 0x34};
    EXPECT_FALSE(crc::verify(two, sizeof(two)));   // 长度刚好等于 CRC 大小
    EXPECT_FALSE(crc::verify(two, 0));
}

// 7. 整帧契约：19 字节四元数帧（0x5A + 16 负载 + 2 CRC），append 后 verify 通过
TEST(CrcFrame, QuaternionFrameRoundTrip) {
    std::vector<std::uint8_t> frame(19, 0x00);
    frame[0] = 0x5A;   // 帧头
    // 负载 frame[1..16] 填点非零值，模拟 4×float32
    for (int i = 1; i <= 16; ++i) {
        frame[i] = static_cast<std::uint8_t>(i * 7);
    }
    crc::append(frame.data(), frame.size());   // CRC 覆盖帧头+负载，写入 frame[17..18]
    EXPECT_TRUE(crc::verify(frame.data(), frame.size()));
}

// 8. 发送帧契约：16 字节目标帧（0xA5 + 标志 + 3×float32 + 2 CRC），append 后 verify 通过
TEST(CrcFrame, TargetFrameRoundTrip) {
    std::vector<std::uint8_t> frame(16, 0x00);
    frame[0] = 0xA5;   // 发送帧头
    frame[1] = 0x01;   // 检测到
    for (int i = 2; i <= 13; ++i) {   // 负载 frame[2..13] 填非零，模拟 3×float32
        frame[i] = static_cast<std::uint8_t>(i * 11);
    }
    crc::append(frame.data(), frame.size());   // CRC 覆盖 frame[0..13]，写入 frame[14..15]
    EXPECT_TRUE(crc::verify(frame.data(), frame.size()));
}
