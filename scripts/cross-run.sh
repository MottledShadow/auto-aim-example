#!/usr/bin/env bash
set -euo pipefail

# 交叉编译识别器调试工具并部署到 Jetson 运行
# 用法：cross-run.sh <camera|file>
mode="${1:-}"
case "$mode" in
    camera|file) ;;
    *) echo "usage: $0 <camera|file>"; exit 1 ;;
esac

target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$project_dir/build/cross"
binary_name="detector_debug_${mode}"
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
# 每次都跑完整流水线：相机内参/畸变(YAML) 与数字分类模型(mlp.onnx+label.txt) 都要送过去，工作目录为 ~/cross-snapshot
scp "$project_dir/config/camera_calibration.yml" "$target:$remote_dir/config/camera_calibration.yml"
scp -r "$project_dir/detector/model" "$target:$remote_dir/model"

echo "Running $binary_name on Jetson..."
if [[ "$mode" == "file" ]]; then
    # file：无 DISPLAY，读 captures/ 的照片，每图一目录输出五阶段标注图到 test_output/gray<阈值>/<图名>/
    ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x $binary_name
./$binary_name captures test_output config/camera_calibration.yml
"
    echo "结果保存在 Jetson: ~/$remote_dir/test_output/gray<阈值>/<图名>/{1_preprocess..5_pnp}.png"
    echo "如需取回开发机: scp -r $target:$remote_dir/test_output ./"
else
    # camera：实时窗口，需要 DISPLAY；窗口内按 1-5 切换/叠加各阶段标注层，q/ESC 退出
    ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x $binary_name

runtime_dir=\"/run/user/\$(id -u)\"

DISPLAY=:0 \
XAUTHORITY=\"\$runtime_dir/gdm/Xauthority\" \
XDG_RUNTIME_DIR=\"\$runtime_dir\" \
./$binary_name
"
fi
