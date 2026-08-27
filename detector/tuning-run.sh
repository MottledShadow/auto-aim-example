#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/detector-tuning"
binary="$build_dir/detector_tuner"
remote_dir="auto-aim"
mode="${1:-}"

if [[ "$mode" != "validate" && "$mode" != "scan" ]]; then
    echo "Usage: $0 validate"
    echo "       $0 scan"
    exit 2
fi

if [[ -n "${2:-}" ]]; then
    echo "The $mode mode takes no additional arguments." >&2
    exit 2
fi

echo "Configuring ARM64 detector tuner build..."
cmake -S "$project_dir/detector/tuning" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake" \
    -DCMAKE_BUILD_TYPE=Release

echo "Building detector_tuner..."
cmake --build "$build_dir" --target detector_tuner --parallel

echo "Deploying detector_tuner..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir\""
scp "$binary" "$target:$remote_dir/detector_tuner"

if [[ "$mode" == "validate" ]]; then
    echo "Validating target-box annotations on Jetson..."
    ssh "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x detector_tuner
./detector_tuner validate captures captures/annotations.csv
"
    exit 0
fi

run_id="$(date -u +%Y%m%d-%H%M%S)"
remote_output="$remote_dir/tuning-results/$run_id"
local_output_root="$project_dir/build/detector-tuning-results"

echo "Running detector parameter search on Jetson..."
ssh "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x detector_tuner
mkdir -p \"tuning-results/$run_id\"
./detector_tuner scan captures captures/annotations.csv \"tuning-results/$run_id\"
"

echo "Copying reports to the development machine..."
mkdir -p "$local_output_root"
scp -r "$target:$remote_output" "$local_output_root/"

echo "Remote results: ~/$remote_output"
echo "Local results:  $local_output_root/$run_id"
