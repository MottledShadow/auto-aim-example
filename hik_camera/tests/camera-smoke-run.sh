#!/usr/bin/env bash
set -euo pipefail

# 海康相机无界面冒烟测试 + 时间戳标定：交叉编译 → 部署到 Jetson → 采帧并写共享配置。
# 前置条件：
#   - 海康 USB 相机已连接并可被 MVS SDK 枚举
#   - SSH 里配好名为 jetson 的主机别名
# 测试至少采 30 帧并覆盖 3 秒；基础采集异常返回非零，时间戳拟合不达标只告警且保留旧配置。
target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/hik_camera_smoke_test"
remote_dir="auto-aim"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building hik_camera_smoke_test..."
cmake --build "$build_dir" --target hik_camera_smoke_test --parallel

echo "Preparing Jetson directory..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir/config\""

echo "Deploying binary..."
scp "$binary" "$target:$remote_dir/hik_camera_smoke_test"

echo "Running headless camera smoke test and timestamp calibration..."
ssh "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x hik_camera_smoke_test
./hik_camera_smoke_test
"

echo "标定通过时结果保存在 Jetson: ~/$remote_dir/config/camera_timestamp.yml"
