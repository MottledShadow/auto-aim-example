#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/armor_pnp_online_test"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building armor pnp online test..."
cmake --build "$build_dir" --target armor_pnp_online_test --parallel

echo "Deploying binary and calibration..."
ssh "$target" 'mkdir -p "$HOME/cross-snapshot/config"'
scp "$binary" "$target:cross-snapshot/armor_pnp_online_test"
# PnP 需要相机内参/畸变，把标定 YAML 一并送过去（程序默认读 config/camera_calibration.yml）
scp "$project_dir/config/camera_calibration.yml" "$target:cross-snapshot/config/camera_calibration.yml"

echo "Running armor pnp online test on Jetson..."
# 实时窗口，需要 DISPLAY；q/ESC 退出
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x armor_pnp_online_test

runtime_dir="/run/user/$(id -u)"

DISPLAY=:0 \
XAUTHORITY="$runtime_dir/gdm/Xauthority" \
XDG_RUNTIME_DIR="$runtime_dir" \
./armor_pnp_online_test
'
