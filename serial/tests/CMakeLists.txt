# serial 测试分两种模式，按「是否作为子目录被包含」区分：
# 1) 独立 project（本机原生跑，crc-test-run.sh）：crc 离线 gtest 单测，不碰 OpenCV/MVCAM，不进交叉构建
# 2) 被交叉构建 add_subdirectory 包含：serial_recv_test 硬件冒烟测试，编进 build/cross、部署到 Jetson 跑

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # === 模式一：独立 project ===
    cmake_minimum_required(VERSION 3.16)

    project(crc_test LANGUAGES CXX)

    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # vendored googletest（third_party/ 已 gitignore），不走 find_package
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/googletest" googletest EXCLUDE_FROM_ALL)

    # 直接编 crc 源，头文件路径指向 serial/inc
    add_executable(crc_test
        crc_test.cpp
        "${CMAKE_CURRENT_SOURCE_DIR}/../src/crc.cpp"
    )

    target_include_directories(crc_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../inc")

    target_link_libraries(crc_test PRIVATE GTest::gtest_main)

    target_compile_options(crc_test PRIVATE -Wall -Wextra -Wpedantic)
else()
    # === 模式二：交叉构建子目录 ===
    # 连真实 STM32 的接收冒烟测试（不是离线单测）：只在交叉构建树里编，跑在 Jetson 上（见 serial/serial-recv-run.sh）
    add_executable(serial_recv_test
        serial_recv_test.cpp
    )

    # 链 serial 即拿到 serial/inc 头文件路径与 Threads
    target_link_libraries(serial_recv_test PRIVATE serial)

    target_compile_options(serial_recv_test PRIVATE -Wall -Wextra -Wpedantic)

    set_target_properties(serial_recv_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
endif()
