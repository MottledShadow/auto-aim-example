#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/armor_offline_test"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building armor offline test..."
cmake --build "$build_dir" --target armor_offline_test --parallel

echo "Deploying binary..."
# 复用 snapshot 的目录，captures/ 里就是刚拍下的照片
ssh "$target" 'mkdir -p "$HOME/cross-snapshot"'
scp "$binary" "$target:cross-snapshot/armor_offline_test"

echo "Running armor offline test on Jetson..."
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x armor_offline_test
# 读 captures/ 的照片，结果输出到 test_output/test_armor_<二值化方法+参数>/
./armor_offline_test captures test_output
'

echo "结果保存在 Jetson: ~/cross-snapshot/test_output/test_armor_<方法+参数>/"
echo "如需取回开发机: scp -r $target:cross-snapshot/test_output ./"
