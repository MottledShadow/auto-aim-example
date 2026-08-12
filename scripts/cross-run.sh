#!/usr/bin/env bash
set -euo pipefail

# 交叉编译分阶段调试测试并部署到 Jetson 运行
# 用法：cross-run.sh <preprocess|lightbar|armor|number|pnp> <offline|online>
stage="${1:-}"
mode="${2:-}"
case "$stage" in
    preprocess|lightbar|armor|number|pnp) ;;
    *) echo "usage: $0 <preprocess|lightbar|armor|number|pnp> <offline|online>"; exit 1 ;;
esac
case "$mode" in
    offline|online) ;;
    *) echo "usage: $0 <preprocess|lightbar|armor|number|pnp> <offline|online>"; exit 1 ;;
esac

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary_name="pipeline_${mode}_test"
binary="$build_dir/$binary_name"
# 复用 snapshot 的目录，captures/ 里就是刚拍下的照片
remote_dir="cross-snapshot"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building $binary_name..."
cmake --build "$build_dir" --target "$binary_name" --parallel

echo "Deploying binary..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir/config\""
scp "$binary" "$target:$remote_dir/$binary_name"
# pnp 需要相机内参/畸变：offline 靠命令行传路径，online 读默认 config/ 路径，两者都要把 YAML 送过去
if [[ "$stage" == "pnp" ]]; then
    scp "$project_dir/config/camera_calibration.yml" "$target:$remote_dir/config/camera_calibration.yml"
fi
# number/pnp 需要数字分类模型：把 model/（mlp.onnx + label.txt）送过去，工作目录为 ~/cross-snapshot
if [[ "$stage" == "number" || "$stage" == "pnp" ]]; then
    scp -r "$project_dir/model" "$target:$remote_dir/model"
fi

echo "Running $binary_name ($stage) on Jetson..."
if [[ "$mode" == "offline" ]]; then
    # 离线：无 DISPLAY，读 captures/ 的照片，结果输出到 test_output/test_<stage>_<方法+参数>/
    ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x $binary_name
./$binary_name $stage captures test_output config/camera_calibration.yml
"
    echo "结果保存在 Jetson: ~/$remote_dir/test_output/test_${stage}_<方法+参数>/"
    echo "如需取回开发机: scp -r $target:$remote_dir/test_output ./"
else
    # 在线：实时窗口，需要 DISPLAY；q/ESC 退出
    ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x $binary_name

runtime_dir=\"/run/user/\$(id -u)\"

DISPLAY=:0 \
XAUTHORITY=\"\$runtime_dir/gdm/Xauthority\" \
XDG_RUNTIME_DIR=\"\$runtime_dir\" \
./$binary_name $stage
"
fi
