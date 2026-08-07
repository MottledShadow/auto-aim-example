set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER
    "$ENV{HOME}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-gcc"
)

set(CMAKE_CXX_COMPILER
    "$ENV{HOME}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-g++"
)

set(CMAKE_SYSROOT "$ENV{HOME}/jetson-sysroot")

# 编译器探测阶段默认会链接一个可执行文件，而交叉链接需要 sysroot 里的 CRT
# 启动文件（Scrt1.o/crti.o，靠下面 -B 指向的 multiarch 目录才找得到）。改成只编译
# 静态库，跳过链接，避免探测阶段因找不到 CRT 而 FATAL_ERROR。
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT
    "-B${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ -isystem ${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu"
)
set(CMAKE_CXX_FLAGS_INIT
    "-B${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/ -isystem ${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu"
)
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-L${CMAKE_SYSROOT}/usr/lib/gcc/aarch64-linux-gnu/11/ -Wl,-rpath-link,${CMAKE_SYSROOT}/lib/aarch64-linux-gnu -Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu"
)

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# FIND_ROOT_PATH_MODE_PACKAGE 为 ONLY，会给 OpenCV_DIR 重新加上 sysroot 前缀再搜索，
# 所以这里写相对 sysroot 根的路径（re-root 后才是真正的 sysroot 内目录）。
set(OpenCV_DIR
    "/usr/lib/cmake/opencv4"
    CACHE PATH "Jetson OpenCV CMake directory"
)

set(MVCAM_ROOT
    "${CMAKE_SYSROOT}/opt/MVS"
    CACHE PATH "Jetson MVS SDK directory"
)
