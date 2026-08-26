#!/usr/bin/env bash
# crc 离线单测：本机 x86 原生编译并运行，不交叉编译、不上 Jetson。
# crc 无硬件依赖，是全项目唯一能脱离 build/cross 单独跑的模块。
set -euo pipefail

# 切到本脚本所在的 serial 目录
cd "$(dirname "$0")"

# 1. 原生配置（独立 project，不碰顶层 OpenCV/MVCAM）
cmake -S tests -B ../build/crc-test -DCMAKE_BUILD_TYPE=Debug

# 2. 编译
cmake --build ../build/crc-test

# 3. 跑测试
../build/crc-test/crc_test
