#!/usr/bin/env bash
set -euo pipefail

# 全链路集成测试：交叉编译 → 部署到 Jetson → 带屏运行
# 链路：海康相机 → 识别器 → 坐标变换(精简追踪器) → 串口发目标帧(0xA5) + 预览窗口画装甲板与坐标
# 前置条件：
#   - 相机接好；串口接到 /dev/ttyTHS1，STM32 按 0x5A 帧发四元数、按 0xA5 16 字节帧收目标
#   - SSH 里配好名为 jetson 的主机别名
#   - 先跑过 tools/cross-handeye-run.sh：标定结果(内参+手眼)会写进共享 ~/auto-aim/config/
#   - 模型已常驻 Jetson ~/auto-aim/model/{mlp.onnx,label.txt}
# detector 按相对 cwd 读 config/camera_calibration.yml 与 model/{mlp.onnx,label.txt}。
# 这些运行时依赖(标定+模型)都常驻 Jetson 共享 config/ 和 model/，本脚本只推二进制，都不推。
target="jetson"
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$project_dir/build/cross"
binary="$build_dir/full_chain_test"
remote_dir="auto-aim"

echo "Configuring ARM64 build..."
cmake -S "$project_dir" -B "$build_dir" \
    -DCMAKE_TOOLCHAIN_FILE="$project_dir/jetson-toolchain.cmake"

echo "Building full_chain_test..."
cmake --build "$build_dir" --target full_chain_test --parallel

echo "Preparing Jetson directory..."
ssh "$target" "mkdir -p \"\$HOME/$remote_dir\""

echo "Deploying binary..."
scp "$binary" "$target:$remote_dir/full_chain_test"

# 标定与模型都不推：config 靠先跑 tools/cross-handeye-run.sh 生成，model 常驻 Jetson。
# 缺了 full_chain_test 会读不到内参/模型起不来，先探测提示。
if ! ssh "$target" "test -f \"\$HOME/$remote_dir/config/camera_calibration.yml\" && test -f \"\$HOME/$remote_dir/model/mlp.onnx\""; then
    echo "  warning: Jetson ~/$remote_dir 缺 config/camera_calibration.yml 或 model/mlp.onnx"
    echo "           config 先跑 tools/cross-handeye-run.sh 标定生成；model 需常驻 ~/$remote_dir/model"
fi

echo "Running full chain on Jetson (窗口显示在 Jetson 屏幕, ESC 退出)..."
echo "  画面: 框出识别到的装甲板(选中块黄色粗线)，标数字/置信度与选中块世界系 x/y/z(mm)"
echo "  左上: send detected 标志 + 发送坐标 + 实时四元数"
echo "  发送: STM32 侧应持续收到 0xA5 16 字节目标帧, CRC 校验通过, 坐标随画面变"
ssh -t "$target" "
set -e
cd \"\$HOME/$remote_dir\"
chmod u+x full_chain_test

runtime_dir=\"/run/user/\$(id -u)\"

DISPLAY=:0 \
XAUTHORITY=\"\$runtime_dir/gdm/Xauthority\" \
XDG_RUNTIME_DIR=\"\$runtime_dir\" \
./full_chain_test
"
