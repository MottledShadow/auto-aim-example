#!/usr/bin/env bash
set -euo pipefail

# 交叉编译串口测试并部署到 Jetson 前台运行
# stdout 经 SSH 回传到开发机，能实时看到输出；Ctrl-C 中断
# 用法：cross-serial-run.sh [recv|echo]
#   recv（默认）：serial_recv_test，收 STM32 四元数帧并打印，会一并送配置 yml
#   echo        ：serial_test，早期链路回显冒烟测试
mode="${1:-recv}"
case "$mode" in
    recv) binary_name="serial_recv_test" ;;
    echo) binary_name="serial_test" ;;
    *) echo "usage: $0 [recv|echo]"; exit 1 ;;
esac

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/$binary_name"
remote_dir="cross-serial"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building $binary_name..."
cmake --build "$build_dir" --target "$binary_name" --parallel

echo "Deploying binary..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir\""
scp "$binary" "$target:$remote_dir/$binary_name"

if [[ "$mode" == "recv" ]]; then
    # recv 要读串口配置，配置路径在代码里写死为 serial/config/serial_config.yml，
    # 按同样的相对路径复刻到 ~/cross-serial/ 下（运行时不再传参）
    ssh "$target" "mkdir -p \"\$HOME/$remote_dir/serial/config\""
    scp "$project_dir/serial/config/serial_config.yml" "$target:$remote_dir/serial/config/serial_config.yml"
fi

echo "Running $binary_name on Jetson (Ctrl-C to stop)..."
# -t 分配伪终端：既能实时看到输出，也能用 Ctrl-C 中断
ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x $binary_name
./$binary_name
"
