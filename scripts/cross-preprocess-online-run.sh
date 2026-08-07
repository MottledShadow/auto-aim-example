#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/preprocess_online_test"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building preprocess online test..."
cmake --build "$build_dir" --target preprocess_online_test --parallel

echo "Deploying binary..."
ssh "$target" 'mkdir -p "$HOME/cross-snapshot"'
scp "$binary" "$target:cross-snapshot/preprocess_online_test"

echo "Running preprocess online test on Jetson..."
# 实时窗口，需要 DISPLAY；q/ESC 退出
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x preprocess_online_test

runtime_dir="/run/user/$(id -u)"

DISPLAY=:0 \
XAUTHORITY="$runtime_dir/gdm/Xauthority" \
XDG_RUNTIME_DIR="$runtime_dir" \
./preprocess_online_test
'
