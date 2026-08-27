#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/detector-tuning"
binary="$build_dir/detector_tuner"
remote_dir="auto-aim"
mode="${1:-}"

if [[ "$mode" != "annotate" && "$mode" != "validate" && "$mode" != "scan" ]]; then
    echo "Usage: $0 annotate [--force]"
    echo "       $0 validate"
    echo "       $0 scan"
    exit 2
fi

annotate_option="${2:-}"
if [[ "$mode" == "annotate" ]]; then
    if [[ -n "$annotate_option" && "$annotate_option" != "--force" ]]; then
        echo "The annotate mode only accepts --force." >&2
        exit 2
    fi
    if [[ -n "${3:-}" ]]; then
        echo "The annotate mode accepts at most one option." >&2
        exit 2
    fi
elif [[ -n "$annotate_option" ]]; then
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

if [[ "$mode" == "annotate" ]]; then
    echo "Selecting target boxes on the Jetson display..."
    ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x detector_tuner

runtime_dir=\"/run/user/\$(id -u)\"

DISPLAY=:0 \\
XAUTHORITY=\"\$runtime_dir/gdm/Xauthority\" \\
XDG_RUNTIME_DIR=\"\$runtime_dir\" \\
./detector_tuner annotate captures captures/annotations.csv $annotate_option
"
    exit 0
fi

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
