# ============================================================
# FindCUDA.cmake — 存根，骗过 OpenCV 的 CUDA 依赖检查
#
# 我们项目不用 CUDA，但 Jetson 上的 OpenCV 编译时带了 CUDA 支持，
# 其 cmake 配置文件内部强制 find_package(CUDA 12.6 REQUIRED)。
# 主力机没有 CUDA toolkit，用这个空壳满足它的变量要求。
# ============================================================

set(CUDA_FOUND TRUE)
set(CUDA_VERSION "12.6")
set(CUDA_VERSION_STRING "12.6")
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(CUDA_NVCC_EXECUTABLE "/usr/local/cuda/bin/nvcc")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
set(CUDA_CUDART_LIBRARY "")
set(CUDA_LIBRARIES "")
set(CUDA_CUBLAS_LIBRARIES "")
