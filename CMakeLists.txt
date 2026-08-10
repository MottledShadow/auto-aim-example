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

# 离线跑 detector 的预处理测试：只读磁盘图片，只依赖 OpenCV，不链接相机 SDK
add_executable(preprocess_offline_test
    tests/preprocess_offline_test.cpp
    detector/detector.cpp
)

target_include_directories(preprocess_offline_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/detector"
)

target_link_libraries(preprocess_offline_test
    PRIVATE
        ${OpenCV_LIBS}
)

target_compile_options(preprocess_offline_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(preprocess_offline_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

# 离线跑 detector 的灯条筛选测试：只读磁盘图片，把各候选的四项筛选指标标到图上，只依赖 OpenCV
add_executable(lightbar_offline_test
    tests/lightbar_offline_test.cpp
    detector/detector.cpp
)

target_include_directories(lightbar_offline_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/detector"
)

target_link_libraries(lightbar_offline_test
    PRIVATE
        ${OpenCV_LIBS}
)

target_compile_options(lightbar_offline_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(lightbar_offline_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)

# 在线跑 detector 的预处理测试：连相机实时取帧 → 预处理 → 双窗口显示（取帧/预处理/显示 各一线程）
add_executable(preprocess_online_test
    tests/preprocess_online_test.cpp
    hik_camera/hik_camera.cpp
    detector/detector.cpp
)

target_include_directories(preprocess_online_test
    PRIVATE
        "${PROJECT_SOURCE_DIR}/hik_camera"
        "${PROJECT_SOURCE_DIR}/detector"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(preprocess_online_test
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(preprocess_online_test PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(preprocess_online_test
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)
