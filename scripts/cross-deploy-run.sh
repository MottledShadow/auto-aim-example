#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$project_dir/build/cross/hik_capture"

echo "Building ARM64 binary..."
cmake --build "$project_dir/build/cross" --parallel

echo "Preparing Jetson directory..."
ssh "$target" 'mkdir -p "$HOME/cross-test/config"'

echo "Deploying binary..."
scp "$binary" "$target:cross-test/hik_capture"

echo "Deploying config..."
rsync -a \
    "$project_dir/config/" \
    "$target:cross-test/config/"

echo "Running on Jetson..."
ssh -t "$target" '
set -e
cd "$HOME/cross-test"
chmod u+x hik_capture

runtime_dir="/run/user/$(id -u)"

DISPLAY=:0 \
XAUTHORITY="$runtime_dir/gdm/Xauthority" \
XDG_RUNTIME_DIR="$runtime_dir" \
./hik_capture --show-binary
'