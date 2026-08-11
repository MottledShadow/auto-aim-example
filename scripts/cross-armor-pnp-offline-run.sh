#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/armor_pnp_offline_test"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building armor pnp offline test..."
cmake --build "$build_dir" --target armor_pnp_offline_test --parallel

echo "Deploying binary and calibration..."
# 复用 snapshot 的目录，captures/ 里就是刚拍下的照片
ssh "$target" 'mkdir -p "$HOME/cross-snapshot/config"'
scp "$binary" "$target:cross-snapshot/armor_pnp_offline_test"
# PnP 需要相机内参/畸变，把标定 YAML 一并送过去
scp "$project_dir/config/camera_calibration.yml" "$target:cross-snapshot/config/camera_calibration.yml"

echo "Running armor pnp offline test on Jetson..."
ssh -t "$target" '
set -e
cd "$HOME/cross-snapshot"
chmod u+x armor_pnp_offline_test
# 读 captures/ 的照片 + config/camera_calibration.yml，结果输出到 test_output/test_pnp_<二值化方法+参数>/
./armor_pnp_offline_test captures test_output config/camera_calibration.yml
'

echo "结果保存在 Jetson: ~/cross-snapshot/test_output/test_pnp_<方法+参数>/"
echo "如需取回开发机: scp -r $target:cross-snapshot/test_output ./"
