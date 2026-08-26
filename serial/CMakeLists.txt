# serial 模块：串口收 STM32 的四元数帧，编成静态库
# serial.hpp 用 PUBLIC 暴露，下游 target 链接 serial 即自动拿到头文件搜索路径
add_library(serial STATIC
    src/serial.cpp
    src/crc.cpp
)

target_include_directories(serial PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc" "${PROJECT_SOURCE_DIR}/common")

# 后台接收线程 → 链 Threads
target_link_libraries(serial PUBLIC Threads::Threads)

target_compile_options(serial PRIVATE -Wall -Wextra -Wpedantic)

# tests/CMakeLists.txt 分两种模式：交叉构建里只编 serial_recv_test 冒烟测试；
# 独立跑（serial/crc-test-run.sh）才是 crc 离线 gtest 单测，不进本交叉构建。
add_subdirectory(tests)
