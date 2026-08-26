#!/usr/bin/env bash
set -euo pipefail

# 交叉编译标定工具并部署到 Jetson 跑手眼标定
# 需要相机(DISPLAY 预览窗口) + 串口 IMU 四元数，两阶段：先内参后手眼
target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/calibrate_camera"
remote_dir="auto-aim"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building calibrate_camera..."
cmake --build "$build_dir" --target calibrate_camera --parallel

echo "Preparing Jetson directory..."
# config/ 要可写：内参与手眼结果都写到这里
ssh "$target" "mkdir -p \"\$HOME/$remote_dir/config\""

echo "Deploying binary..."
scp "$binary" "$target:$remote_dir/calibrate_camera"

echo "Running hand-eye calibration on Jetson (窗口显示在 Jetson 屏幕)..."
echo "  阶段一: SPACE 采纳棋盘视图, u 撤销, 采够后 ENTER 算内参"
echo "  阶段二: 每次采集前转动云台改变朝向, SPACE 采纳姿态, u 撤销, ENTER 解算, ESC 退出"
ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x calibrate_camera

runtime_dir=\"/run/user/\$(id -u)\"

DISPLAY=:0 \
XAUTHORITY=\"\$runtime_dir/gdm/Xauthority\" \
XDG_RUNTIME_DIR=\"\$runtime_dir\" \
./calibrate_camera
"

echo "标定结果在 Jetson 共享 config: ~/$remote_dir/config/{camera_calibration,hand_eye_calibration}.yml"
echo "full-chain-run.sh 直接读这份共享 config，无需取回开发机再推"
