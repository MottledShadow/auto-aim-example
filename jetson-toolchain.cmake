# ============================================================
# jetson-toolchain.cmake — 主力机交叉编译 Jetson (aarch64) 二进制
#
# 用法:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=jetson-toolchain.cmake
#   cmake --build build
#
# 前提:
#   1. 已安装交叉编译器: gcc-aarch64-linux-gnu / g++-aarch64-linux-gnu
#   2. 已从 Jetson 拉好 sysroot 到 ~/jetson-sysroot/
# ============================================================

# ---- 第 0 部分：骗过 OpenCV 内部绑定的 CUDA 依赖 ----
# Jetson 的 OpenCV 编译时带了 CUDA，其 cmake 配置文件内部强制
# find_host_package(CUDA 12.6 EXACT REQUIRED)。主力机没有 CUDA toolkit，
# 提前设好变量让它跳过检测。
set(CUDA_FOUND TRUE CACHE BOOL "" FORCE)
set(CUDA_VERSION "12.6" CACHE STRING "" FORCE)
set(CUDA_VERSION_STRING "12.6" CACHE STRING "" FORCE)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda" CACHE PATH "" FORCE)
set(CUDA_NVCC_EXECUTABLE "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "" FORCE)
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include" CACHE PATH "" FORCE)
set(CUDA_CUDART_LIBRARY "" CACHE FILEPATH "" FORCE)
set(CUDA_LIBRARIES "" CACHE STRING "" FORCE)
set(CUDA_CUBLAS_LIBRARIES "" CACHE STRING "" FORCE)
set(CUDA_nppc_LIBRARY "" CACHE FILEPATH "" FORCE)
set(CUDA_nppi_LIBRARY "" CACHE FILEPATH "" FORCE)
set(CUDA_npps_LIBRARY "" CACHE FILEPATH "" FORCE)

# OpenCV 内部调用了 FindCUDA 提供的 find_cuda_helper_libs 宏。
# 因为我们跳过了 FindCUDA 的加载，需要自己补上这个宏。
macro(find_cuda_helper_libs _name)
  if(NOT CUDA_${_name}_LIBRARY)
    set(CUDA_${_name}_LIBRARY "" CACHE FILEPATH "" FORCE)
  endif()
endmacro()

# ---- 第 1 部分：目标系统 ----
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ---- 第 2 部分：交叉编译器 ----
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# ---- 第 3 部分：sysroot ----
# 告诉编译器和链接器：去这个目录找头文件和库
# 而不是你主力机自己的 /usr/include 和 /usr/lib
set(CMAKE_SYSROOT "$ENV{HOME}/jetson-sysroot")

# ---- 第 4 部分：搜索策略 ----
# 所有 find_* 命令（find_library, find_path, find_package 等）
# 都只在 sysroot 里搜，不碰主力机自己的库，防止混入 x86 的 .so
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # 程序（如 cmake 自己）从主机找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)    # 库文件 只从 sysroot 找
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)    # 头文件 只从 sysroot 找
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)    # cmake 包 只从 sysroot 找

# ---- 第 5 部分：额外搜索路径 ----
# OpenCV 装在了 /usr/local 而不是 /usr，需要显式加进去
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# ---- 第 6 部分：MVS SDK 路径覆盖 ----
# 交叉编译时，MVS 的头文件和库在 sysroot 里
# CMakeLists.txt 默认 MVCAM_ROOT=/opt/MVS，编译器不会自动加 sysroot 前缀
# 这里覆盖为 sysroot 里的路径
set(MVCAM_ROOT "${CMAKE_SYSROOT}/opt/MVS" CACHE PATH "" FORCE)

# ---- 第 7 部分：pkg-config ----
# 如果 Jetson 上有通过 pkg-config 管理的库，这里确保它搜 sysroot
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
set(ENV{PKG_CONFIG_PATH}        "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/local/lib/pkgconfig")
