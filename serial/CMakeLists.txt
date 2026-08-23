# serial 模块：串口收 STM32 的四元数帧，编成静态库
# serial.hpp 用 PUBLIC 暴露，下游 target 链接 serial 即自动拿到头文件搜索路径
add_library(serial STATIC
    serial.cpp
)

target_include_directories(serial PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# 读配置用 cv::FileStorage → 链 OpenCV；后台接收线程 → 链 Threads
target_link_libraries(serial PUBLIC ${OpenCV_LIBS} Threads::Threads)

target_compile_options(serial PRIVATE -Wall -Wextra -Wpedantic)

# 连真实 STM32 的接收冒烟测试（不是离线单测）
add_executable(serial_recv_test
    serial_recv_test.cpp
)

target_link_libraries(serial_recv_test PRIVATE serial)

target_compile_options(serial_recv_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(serial_recv_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

# 早期链路回显冒烟测试：STM32 发、Jetson 原样回发，纯 POSIX termios、不依赖任何库
add_executable(serial_test
    serial_test.cpp
)

target_compile_options(serial_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(serial_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
