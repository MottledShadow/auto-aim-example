# detector 模块：预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP，编成一个静态库
# inc/ 用 PUBLIC 暴露，下游 target 链接 detector 即自动拿到头文件搜索路径
add_library(detector STATIC
    src/lightbar_detector.cpp
    src/pnp_solver.cpp
    src/number_classifier.cpp
    src/detector.cpp
)

target_include_directories(detector PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc" "${PROJECT_SOURCE_DIR}/common")

target_link_libraries(detector PUBLIC ${OpenCV_LIBS} Threads::Threads)

target_compile_options(detector PRIVATE -Wall -Wextra -Wpedantic)
