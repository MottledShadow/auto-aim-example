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

# 连真实 STM32 的收发冒烟测试（不是离线单测）：只在交叉构建树里编，跑在 Jetson 上（见 serial/serial-io-smoke-run.sh）
add_executable(serial_io_smoke_test
    tests/serial_io_smoke_test.cpp
)

target_link_libraries(serial_io_smoke_test PRIVATE serial)

target_compile_options(serial_io_smoke_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(serial_io_smoke_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

# crc 离线单测是 tests/ 下的独立 project，本机原生跑（见 serial/crc-test-run.sh），不进本交叉构建。
