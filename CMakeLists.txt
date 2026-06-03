cmake_minimum_required(VERSION 3.16)

project(auto_aim_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(TARGET_NAME hik_capture)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)

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

add_executable(${TARGET_NAME}
    src/main.cpp
    src/hik_capture.cpp
    src/app_options.cpp
    src/debug_draw.cpp
    src/vision_pipeline.cpp
    src/armor_preprocessor.cpp
    src/light_bar_filter.cpp
    src/armor_matcher.cpp
    src/pnp_solver.cpp
)

target_include_directories(${TARGET_NAME}
    PRIVATE
        "${PROJECT_SOURCE_DIR}/include"
        "${MVCAM_INCLUDE_DIR}"
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
        ${OpenCV_LIBS}
        "${MVCAM_LIBRARY}"
)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /W4)
else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

set_target_properties(${TARGET_NAME}
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
        BUILD_RPATH "${MVCAM_LIBRARY_DIR}"
        INSTALL_RPATH "${MVCAM_LIBRARY_DIR}"
)

add_custom_command(
    TARGET ${TARGET_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/config"
            "$<TARGET_FILE_DIR:${TARGET_NAME}>/config"
)
