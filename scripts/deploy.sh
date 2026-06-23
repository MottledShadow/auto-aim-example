#!/usr/bin/env bash
# ============================================================
# deploy.sh — 从主力机 WSL 部署到 Jetson
#
# 用法:
#   ./scripts/deploy.sh remote          rsync 源码 + Jetson 编译 + 运行
#   ./scripts/deploy.sh remote build    rsync 源码 + Jetson 编译
#   ./scripts/deploy.sh remote sync     仅 rsync 源码
#
#   ./scripts/deploy.sh cross           本地交叉编译 + 推二进制 + 运行
#   ./scripts/deploy.sh cross build     本地交叉编译 + 推二进制
#
# 前提:
#   1. WSL 已配好免密 SSH 到 Jetson
#   2. Jetson 上有 CMake / OpenCV / MVS SDK
#   3. (cross 模式) WSL 已装交叉编译器 + 已拉 sysroot
# ============================================================
set -euo pipefail

# ---------- 配置 ----------
JETSON_USER="nvidia"
JETSON_IP="10.1.102.55"
JETSON_PROJECT_DIR="/home/${JETSON_USER}/auto_aim_example"
LOCAL_PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOCAL_CROSS_BUILD_DIR="${LOCAL_PROJECT_DIR}/build-jetson"
TOOLCHAIN_FILE="${LOCAL_PROJECT_DIR}/jetson-toolchain.cmake"

# ---------- 颜色 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERR]${NC}   $*"; }
step()  { echo -e "\n${GREEN}>>>${NC} $*"; }

# ============================================================
# 远程编译模式 （rsync 源码 → Jetson 上 cmake/make）
# ============================================================

sync_code() {
    step "rsync 源码到 Jetson..."
    # rsync 只传差异部分，跳过已有文件
    # --delete 让 Jetson 端删除主力机已删掉的文件，保持两边一致
    rsync -avz \
        --delete \
        --exclude '.git/' \
        --exclude 'build/' \
        --exclude 'build-jetson/' \
        --exclude 'cmake-build-*/' \
        "${LOCAL_PROJECT_DIR}/" \
        "${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}/"
}

build_on_jetson() {
    step "Jetson 远程编译..."
    # bash -s << 'BUILD_SCRIPT' 是把下面整段脚本传给 Jetson 的 bash
    # 单引号 'BUILD_SCRIPT' 防止本地 bash 展开 $ 变量
    ssh "${JETSON_USER}@${JETSON_IP}" bash -s << 'BUILD_SCRIPT'
set -euo pipefail
PROJECT_DIR="$HOME/auto_aim_example"
BUILD_DIR="$PROJECT_DIR/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "[CMAKE] Configuring..."
cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -20

NPROC=$(nproc)
echo "[MAKE] Building with $NPROC jobs..."
make -j"$NPROC" 2>&1

echo ""
echo "  Build SUCCESS → $BUILD_DIR/hik_capture"
BUILD_SCRIPT
}

# ============================================================
# 交叉编译模式（主力机编译 arm64 → 推二进制到 Jetson）
# ============================================================

cross_compile() {
    step "本地交叉编译..."
    mkdir -p "${LOCAL_CROSS_BUILD_DIR}"

    cmake -B "${LOCAL_CROSS_BUILD_DIR}" \
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
        -DCMAKE_BUILD_TYPE=Release

    cmake --build "${LOCAL_CROSS_BUILD_DIR}" -- -j"$(nproc)"

    info "二进制产出: ${LOCAL_CROSS_BUILD_DIR}/hik_capture"
}

push_binary() {
    step "推送二进制到 Jetson..."
    ssh "${JETSON_USER}@${JETSON_IP}" "mkdir -p ${JETSON_PROJECT_DIR}/build"
    scp "${LOCAL_CROSS_BUILD_DIR}/hik_capture" \
        "${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}/build/"
}

push_config() {
    step "推送 config 目录..."
    rsync -avz \
        "${LOCAL_PROJECT_DIR}/config/" \
        "${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}/config/"
}

# ============================================================
# 运行
# ============================================================

run_on_jetson() {
    step "Jetson 运行..."
    # DISPLAY=:0 让画面输出到 Jetson 自己的屏幕
    # -t 分配伪终端，方便 Ctrl+C 中断
    ssh -t "${JETSON_USER}@${JETSON_IP}" \
        "export DISPLAY=:0 && cd ${JETSON_PROJECT_DIR}/build && ./hik_capture $*"
}

# ============================================================
# 帮助
# ============================================================

usage() {
    echo ""
    echo "用法: $0 <模式> [动作] [-- 程序参数...]"
    echo ""
    echo "模式:"
    echo "  remote     rsync 源码 + Jetson 编译"
    echo "  cross      主力机交叉编译 + 推二进制到 Jetson"
    echo ""
    echo "动作 (remote):"
    echo "  run        推送 + 编译 + 运行（默认）"
    echo "  build      推送 + 编译"
    echo "  sync       仅推送源码"
    echo ""
    echo "动作 (cross):"
    echo "  run        编译 + 推二进制 + 推 config + 运行（默认）"
    echo "  build      编译 + 推二进制 + 推 config"
    echo ""
    echo "程序参数 (传给 hik_capture):"
    echo "  --show-binary        显示二值化中间结果（默认显示原图预览）"
    echo "  --help               显示 hik_capture 帮助"
    echo ""
    echo "示例:"
    echo "  $0 cross                         交叉编译 + 推送 + 实时预览"
    echo "  $0 cross run --show-binary       交叉编译 + 二值化预览"
    echo "  $0 cross build                   仅编译 + 推送，不运行"
    echo "  $0 remote                        远程编译 + 运行"
    echo ""
}

# ============================================================
# 主入口
# ============================================================

main() {
    local mode="${1:-}"
    local action="${2:-run}"
    shift 2 2>/dev/null || true   # 剩下的参数传给 hik_capture

    if [[ -z "$mode" ]] || [[ "$mode" == "-h" ]] || [[ "$mode" == "--help" ]]; then
        usage
        exit 0
    fi

    info "本地项目: ${LOCAL_PROJECT_DIR}"
    info "目标:     ${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}"
    if [[ $# -gt 0 ]]; then
        info "程序参数: $*"
    fi

    case "$mode" in
        remote)
            case "$action" in
                sync)
                    sync_code
                    info "源码已推送（未编译）"
                    ;;
                build)
                    sync_code
                    build_on_jetson
                    ;;
                run)
                    sync_code
                    build_on_jetson
                    run_on_jetson "$@"
                    ;;
                *)
                    err "未知动作: $action"
                    usage
                    exit 1
                    ;;
            esac
            ;;
        cross)
            case "$action" in
                build)
                    cross_compile
                    push_binary
                    push_config
                    info "交叉编译 + 推送完成"
                    ;;
                run)
                    cross_compile
                    push_binary
                    push_config
                    run_on_jetson "$@"
                    ;;
                *)
                    err "未知动作: $action"
                    usage
                    exit 1
                    ;;
            esac
            ;;
        *)
            err "未知模式: $mode"
            usage
            exit 1
            ;;
    esac
}

main "$@"
