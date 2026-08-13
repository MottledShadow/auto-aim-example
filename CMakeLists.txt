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

# detector 识别模块（静态库），头文件路径经 PUBLIC 传递给下游 target
add_subdirectory(detector)

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
    tools/snapshot.cpp
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

add_executable(calibrate_camera
    tools/calibrate_camera.cpp
    hik_camera/hik_camera.cpp
)

target_include_directories(calibrate_camera
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(calibrate_camera
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(calibrate_camera PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(calibrate_camera
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)

# 识别器离线调试工具（读磁盘图片）：一次跑完整流水线、每图一目录输出五阶段标注图，只依赖 OpenCV，不链接相机 SDK
add_executable(detector_debug_file
    tests/detector_debug_file.cpp
    debug_draw/debug_draw.cpp
)

target_include_directories(detector_debug_file
    PRIVATE
        "${PROJECT_SOURCE_DIR}/debug_draw"
)

target_link_libraries(detector_debug_file
    PRIVATE
        detector
        ${OpenCV_LIBS}
)

target_compile_options(detector_debug_file PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(detector_debug_file
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

# 识别器在线调试工具（接相机实时）：每帧跑完整流水线 → 单窗口按 1-5 键切换/叠加各阶段标注层（取帧/处理/显示 各一线程）
add_executable(detector_debug_camera
    tests/detector_debug_camera.cpp
    hik_camera/hik_camera.cpp
    debug_draw/debug_draw.cpp
)

target_include_directories(detector_debug_camera
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${PROJECT_SOURCE_DIR}/debug_draw"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(detector_debug_camera
    PRIVATE
        detector
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(detector_debug_camera PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(detector_debug_camera
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)
