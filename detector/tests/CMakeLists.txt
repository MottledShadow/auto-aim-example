cmake_minimum_required(VERSION 3.16)

project(lightbar_detector_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Use the vendored GoogleTest so the cross build does not need a target-side
# package installation.
add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/googletest"
    googletest
    EXCLUDE_FROM_ALL
)

add_executable(lightbar_detector_test
    lightbar_detector_test.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/lightbar_detector.cpp"
)

target_include_directories(lightbar_detector_test
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../inc"
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lightbar_detector_test
    PRIVATE
        GTest::gtest_main
        ${OpenCV_LIBS}
)

target_compile_options(lightbar_detector_test PRIVATE -Wall -Wextra -Wpedantic)
