#!/usr/bin/env bash
set -euo pipefail

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/calibrate_camera"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building calibration tool..."
cmake --build "$build_dir" --target calibrate_camera --parallel

echo "Deploying calibration tool..."
ssh "$target" 'mkdir -p "$HOME/cross-calibration/config"'
scp "$binary" "$target:cross-calibration/calibrate_camera"

echo "Starting calibration on Jetson..."
ssh -t "$target" '
set -u
cd "$HOME/cross-calibration"
chmod u+x calibrate_camera

runtime_dir="/run/user/$(id -u)"

DISPLAY=:0 \
XAUTHORITY="$runtime_dir/gdm/Xauthority" \
XDG_RUNTIME_DIR="$runtime_dir" \
./calibrate_camera </dev/null &
pid=$!
cancelled=0

stop_process() {
    kill -TERM "$pid" 2>/dev/null || true
    exit 130
}
trap stop_process HUP INT TERM

echo "Control from this terminal: SPACE capture, U undo, ENTER calibrate, Q quit"
while kill -0 "$pid" 2>/dev/null; do
    if ! IFS= read -rsn1 -t 1 key; then
        continue
    fi
    case "$key" in
        " ") kill -USR1 "$pid" ;;
        u|U) kill -USR2 "$pid" ;;
        "") kill -QUIT "$pid" ;;
        q|Q)
            cancelled=1
            kill -TERM "$pid"
            break
            ;;
    esac
done

if wait "$pid"; then
    status=0
else
    status=$?
fi
trap - HUP INT TERM

if ((cancelled)); then
    exit 2
fi
exit "$status"
'

echo "Downloading calibration result..."
scp \
    "$target:cross-calibration/config/camera_calibration.yml" \
    "$project_dir/config/camera_calibration.yml"

echo "Saved calibration to $project_dir/config/camera_calibration.yml"
