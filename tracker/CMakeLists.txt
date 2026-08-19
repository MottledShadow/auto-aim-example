# tracker 模块：接 detector 的相机系装甲板，转到世界系(FLU)精简装甲板，编成一个静态库
# inc/ 用 PUBLIC 暴露；链接 detector 拿 Armor/ArmorType，OpenCV + Eigen 做位姿运算
add_library(tracker STATIC
    src/tracker.cpp
)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

target_include_directories(tracker PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(tracker PUBLIC detector ${OpenCV_LIBS} Eigen3::Eigen)

target_compile_options(tracker PRIVATE -Wall -Wextra -Wpedantic)
