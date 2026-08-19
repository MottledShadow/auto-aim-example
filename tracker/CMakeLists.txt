# tracker 模块：接 detector 的相机系装甲板，转到世界系(FLU)精简装甲板，编成一个静态库
# inc/ 用 PUBLIC 暴露；链接 detector 拿 Armor/ArmorType，OpenCV + Eigen 做位姿运算
# 三个阶段各一源：坐标变换 / 状态估计(临时低通，以后换 EKF) / facade
add_library(tracker STATIC
    src/coordinate_transform.cpp
    src/target_estimator.cpp
    src/tracker.cpp
)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

target_include_directories(tracker PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

# 公共头暴露 cv::* / Armor，故 OpenCV/detector 走 PUBLIC；Eigen 只在 coordinate_transform.cpp 内部用，PRIVATE 即可
target_link_libraries(tracker PUBLIC detector ${OpenCV_LIBS} PRIVATE Eigen3::Eigen)

target_compile_options(tracker PRIVATE -Wall -Wextra -Wpedantic)
