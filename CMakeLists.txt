cmake_minimum_required(VERSION 3.16)

project(auto_aim_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d dnn)
find_package(Threads REQUIRED)

set(MVCAM_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root directory")
set(MVCAM_INCLUDE_DIR "${MVCAM_ROOT}/include" CACHE PATH "Hikrobot MVS SDK include directory")

if(DEFINED ENV{MVCAM_COMMON_RUNENV} AND NOT "$ENV{MVCAM_COMMON_RUNENV}" STREQUAL "")
    set(MVCAM_LIBRARY_DIR "$ENV{MVCAM_COMMON_RUNENV}/${CMAKE_SYSTEM_PROCESSOR}" CACHE PATH "Hikrobot MVS SDK library directory")
else()
    set(MVCAM_LIBRARY_DIR "${MVCAM_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}" CACHE PATH "Hikrobot MVS SDK library directory")
endif()

find_library(
    MVCAM_LIBRARY
    NAMES MvCameraControl
    PATHS "${MVCAM_LIBRARY_DIR}"
    NO_DEFAULT_PATH
)

if(NOT MVCAM_LIBRARY)
    message(FATAL_ERROR
        "Cannot find MvCameraControl in ${MVCAM_LIBRARY_DIR}. "
        "Set MVCAM_ROOT or MVCAM_LIBRARY_DIR when configuring CMake.")
endif()

# hik_camera 相机模块（静态库）：后台采集线程驱动，MVCAM/OpenCV 经 PUBLIC 传给下游
add_subdirectory(hik_camera)

# detector 识别模块（静态库），头文件路径经 PUBLIC 传递给下游 target
add_subdirectory(detector)

# tracker 追踪模块（静态库）：相机系装甲板 → 世界系精简装甲板，依赖 detector
add_subdirectory(tracker)

# serial 模块（静态库）：串口收 STM32 四元数，附带 serial_recv_test / serial_test 两个冒烟测试
add_subdirectory(serial)

# 手眼标定用 Eigen 把 IMU 四元数转旋转矩阵，链 serial 取云台朝向（serial 以 PUBLIC 暴露头文件路径）
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

add_executable(calibrate_camera
    tools/calibrate_camera.cpp
)

# hik_camera(PUBLIC) 传来 hik_camera/inc + MVCAM include/lib，serial(PUBLIC) 传来 serial/inc
target_link_libraries(calibrate_camera
    PRIVATE
        hik_camera
        serial
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

target_compile_options(calibrate_camera PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(calibrate_camera
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)
