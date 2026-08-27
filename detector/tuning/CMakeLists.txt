cmake_minimum_required(VERSION 3.16)

project(detector_tuner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(detector_tuner
    detector_tuner.cpp
    tuning_core.cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../src/lightbar_detector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../debug_draw/src/debug_draw.cpp"
)

target_include_directories(detector_tuner
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../inc"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../debug_draw/inc"
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(detector_tuner PRIVATE ${OpenCV_LIBS})

target_compile_options(detector_tuner PRIVATE -Wall -Wextra -Wpedantic)
