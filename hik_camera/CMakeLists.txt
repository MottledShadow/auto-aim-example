# hik_camera 模块：海康相机后台采集线程驱动（LatestImagesOnly），编成静态库
# 公开头 hik_camera.hpp 暴露了 MvCameraControl.h 与 OpenCV，故这两者走 PUBLIC 传给下游
add_library(hik_camera STATIC
    src/hik_camera.cpp
)

target_include_directories(hik_camera
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/inc"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(hik_camera
    PUBLIC
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
        Threads::Threads
)

target_compile_options(hik_camera PRIVATE -Wall -Wextra -Wpedantic)
