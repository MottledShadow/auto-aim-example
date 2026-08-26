# crc 离线单测：独立 project，本机 x86 原生编译（见 serial/crc-test-run.sh），
# 不进交叉构建、不碰顶层 OpenCV/MVCAM，也从不被 add_subdirectory 包含。
# crc 是全项目唯一无硬件依赖的部分，所以能脱离交叉编译单独跑。
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
