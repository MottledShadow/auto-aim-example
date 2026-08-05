#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/detector_test"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building detector test..."
cmake --build "$build_dir" --target detector_test --parallel

echo "Deploying binary..."
# 复用 snapshot 的目录，captures/ 里就是刚拍下的照片
ssh "$target" 'mkdir -p "$HOME/cross-snapshot"'
scp "$binary" "$target:cross-snapshot/detector_test"

echo "Running detector test on Jetson..."
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x detector_test
# 读 captures/ 的照片，结果输出到 test_output/test_<时间戳>/
./detector_test captures test_output
'

echo "结果保存在 Jetson: ~/cross-snapshot/test_output/test_<时间戳>/"
echo "如需取回开发机: scp -r $target:cross-snapshot/test_output ./"
