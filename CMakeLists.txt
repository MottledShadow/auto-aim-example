cmake_minimum_required(VERSION 3.16)

project(auto_aim_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
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

add_executable(hik_camera_test
    tests/hik_camera_test.cpp
    hik_camera/hik_camera.cpp
)

target_include_directories(hik_camera_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(hik_camera_test
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(hik_camera_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(hik_camera_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)

add_executable(snapshot
    tools/snapshot_main.cpp
    hik_camera/hik_camera.cpp
)

target_include_directories(snapshot
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(snapshot
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(snapshot PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(snapshot
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)

# 离线分阶段调试测试：读磁盘图片，命令行 <stage> 选阶段(preprocess/lightbar/armor/pnp)，只依赖 OpenCV，不链接相机 SDK
add_executable(pipeline_offline_test
    tests/pipeline_offline_test.cpp
    detector/detector.cpp
    detector/detector_pnp.cpp
    debug_draw/debug_draw.cpp
)

target_include_directories(pipeline_offline_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/detector"
        "${PROJECT_SOURCE_DIR}/debug_draw"
)

target_link_libraries(pipeline_offline_test
    PRIVATE
        ${OpenCV_LIBS}
)

target_compile_options(pipeline_offline_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(pipeline_offline_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

# 在线分阶段调试测试：连相机实时取帧，命令行 <stage> 选阶段 → 单窗口显示（取帧/处理/显示 各一线程）
add_executable(pipeline_online_test
    tests/pipeline_online_test.cpp
    hik_camera/hik_camera.cpp
    detector/detector.cpp
    detector/detector_pnp.cpp
    debug_draw/debug_draw.cpp
)

target_include_directories(pipeline_online_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${PROJECT_SOURCE_DIR}/detector"
        "${PROJECT_SOURCE_DIR}/debug_draw"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(pipeline_online_test
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(pipeline_online_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(pipeline_online_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)
