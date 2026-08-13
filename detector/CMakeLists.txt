# detector 模块：预处理 → 灯条筛选 → 装甲配对 → 数字分类 → PnP，编成一个静态库
# inc/ 用 PUBLIC 暴露，下游 target 链接 detector 即自动拿到头文件搜索路径
add_library(detector STATIC
    src/detector.cpp
    src/pnp.cpp
    src/classifier.cpp
)

target_include_directories(detector PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(detector PUBLIC ${OpenCV_LIBS})

target_compile_options(detector PRIVATE -Wall -Wextra -Wpedantic)
