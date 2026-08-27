#!/usr/bin/env bash
set -euo pipefail

# Lightbar matcher unit tests: cross-compile for ARM64, deploy, then run on Jetson.
# These tests use synthetic geometry and require no camera, model, or calibration files.
target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/detector-test"
binary="$build_dir/lightbar_detector_test"
remote_dir="auto-aim"

echo "Configuring ARM64 detector test build..."
cmake -S "$project_dir/detector/tests" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building lightbar_detector_test..."
cmake --build "$build_dir" --target lightbar_detector_test --parallel

echo "Preparing Jetson directory..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir\""

echo "Deploying lightbar_detector_test..."
scp "$binary" "$target:$remote_dir/lightbar_detector_test"

echo "Running lightbar detector tests..."
ssh "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x lightbar_detector_test
./lightbar_detector_test
"
