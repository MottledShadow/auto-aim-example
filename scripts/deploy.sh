#!/usr/bin/env bash
# ============================================================
# deploy.sh — 从主力机 WSL 推送代码到 Jetson 并编译
#
# 用法:
#   ./scripts/deploy.sh          # 推送 + 编译
#   ./scripts/deploy.sh run      # 推送 + 编译 + 运行
#   ./scripts/deploy.sh sync     # 仅推送，不编译
#
# 前提:
#   1. WSL 已配好免密 SSH 到 Jetson
#   2. Jetson 上有 CMake / OpenCV / MVS SDK
# ============================================================
set -euo pipefail

# ---------- 配置（按你的环境改） ----------
JETSON_USER="nvidia"
JETSON_IP="10.1.102.55"
JETSON_PROJECT_DIR="/home/${JETSON_USER}/auto_aim_example"
LOCAL_PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ---------- 颜色输出 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
err()   { echo -e "${RED}[ERR]${NC}   $*"; }

# ---------- 第 1 步：rsync 推送代码 ----------
sync_code() {
    info "同步代码到 Jetson..."
    rsync -avz \
        --delete \
        --exclude '.git/' \
        --exclude 'build/' \
        --exclude 'cmake-build-*/' \
        "${LOCAL_PROJECT_DIR}/" \
        "${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}/"
}

# ---------- 第 2 步：远程编译 ----------
build_on_jetson() {
    info "在 Jetson 上编译..."
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
echo "========================================="
echo "  Build SUCCESS"
echo "  Binary: $BUILD_DIR/hik_capture"
echo "========================================="
BUILD_SCRIPT
}

# ---------- 第 3 步：远程运行 ----------
run_on_jetson() {
    info "在 Jetson 上运行..."
    ssh -t "${JETSON_USER}@${JETSON_IP}" \
        "cd ${JETSON_PROJECT_DIR}/build && ./hik_capture"
}

# ---------- 主入口 ----------
main() {
    local mode="${1:-build}"

    info "本地项目: ${LOCAL_PROJECT_DIR}"
    info "目标:     ${JETSON_USER}@${JETSON_IP}:${JETSON_PROJECT_DIR}"
    echo ""

    case "$mode" in
        sync)
            sync_code
            info "代码已推送（未编译）"
            ;;
        build)
            sync_code
            build_on_jetson
            ;;
        run)
            sync_code
            build_on_jetson
            run_on_jetson
            ;;
        *)
            err "未知模式: $mode"
            echo "用法: $0 {sync|build|run}"
            exit 1
            ;;
    esac
}

main "$@"
