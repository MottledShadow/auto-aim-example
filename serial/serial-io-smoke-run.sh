#!/usr/bin/env bash
set -euo pipefail

# STM32↔Jetson 串口收发硬件冒烟测试：交叉编译 → 部署到 Jetson → 每 100ms 打印最新四元数、并回发一帧目标
# 前置条件：
#   - STM32 已上电，按 19 字节帧格式(0x5A + 4×float32 小端 + CRC16)持续发四元数
#   - STM32 侧能按 16 字节帧格式(0xA5 + 检测标志 + 3×float32 小端 x/y/z + CRC16)收目标
#   - 串口接到 Jetson 的 /dev/ttyTHS1，波特率与 SerialConfig 默认值(460800)一致
#   - SSH 里配好名为 jetson 的主机别名
# 设备/波特率写死在 serial/inc/serial.hpp 的 SerialConfig，改这些改头文件，命令行不传参。
target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/serial_io_smoke_test"
remote_dir="auto-aim"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building serial_io_smoke_test..."
cmake --build "$build_dir" --target serial_io_smoke_test --parallel

echo "Preparing Jetson directory..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir\""

echo "Deploying binary..."
scp "$binary" "$target:$remote_dir/serial_io_smoke_test"

echo "Running serial smoke test on Jetson (Ctrl-C 退出)..."
echo "  收: 每 100ms 一行 recv w= x= y= z=，数值随云台姿态变、模长≈1"
echo "  发: 同时每 100ms 回发一帧 0xA5 目标帧(标志=1, x/y/z=1/2/3)，对端应校验通过"
echo "  异常: 一直 recv w=1 x=0 y=0 z=0 不动 → 查接线 / 波特率 / 帧格式"
ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x serial_io_smoke_test
./serial_io_smoke_test
"
