set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER
    "$ENV{HOME}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-gcc"
)

set(CMAKE_CXX_COMPILER
    "$ENV{HOME}/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-g++"
)

set(CMAKE_SYSROOT "$ENV{HOME}/jetson-sysroot")
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

set(OpenCV_DIR
    "${CMAKE_SYSROOT}/usr/lib/cmake/opencv4"
    CACHE PATH "Jetson OpenCV CMake directory"
)

set(MVCAM_ROOT
    "${CMAKE_SYSROOT}/opt/MVS"
    CACHE PATH "Jetson MVS SDK directory"
)
