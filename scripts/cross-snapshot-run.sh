#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/snapshot"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building snapshot tool..."
cmake --build "$build_dir" --target snapshot --parallel

echo "Preparing Jetson directory..."
ssh "$target" 'mkdir -p "$HOME/cross-snapshot/captures"'

echo "Deploying binary..."
scp "$binary" "$target:cross-snapshot/snapshot"

echo "Running on Jetson (窗口显示在 Jetson 屏幕: SPACE 拍照, q 退出)..."
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x snapshot

runtime_dir="/run/user/$(id -u)"

DISPLAY=:0 \
XAUTHORITY="$runtime_dir/gdm/Xauthority" \
XDG_RUNTIME_DIR="$runtime_dir" \
./snapshot
'

echo "拍下的照片保存在 Jetson: ~/cross-snapshot/captures/"
