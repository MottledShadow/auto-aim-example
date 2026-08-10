#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "detector.hpp"

//与 detector.cpp 里 filterLightBars 一致的退化轮廓判定阈值
constexpr double kEpsilon = 1e-6;

//把一个 double 转成固定小数位的短字符串，用来标注
std::string fmt(double value, int decimals)
{
    std::ostringstream oss;
    oss.precision(decimals);
    oss << std::fixed << value;
    return oss.str();
}

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录（可用命令行覆盖，默认对应部署脚本的约定）
    const std::string input_dir = (argc > 1) ? argv[1] : "captures";
    const std::string output_root = (argc > 2) ? argv[2] : "test_output";

    //预处理参数（默认通道相减+红）和待调的灯条筛选参数，均走头文件默认值
    //要测灰度或蓝色，直接改这里的 pre_params.method / target_color
    auto_aim::PreprocessParams pre_params;
    auto_aim::LightBarFilterParams filter_params;

    //输出文件夹名标明二值化方法 + 参数，比时间戳直观
    std::string method_tag;
    if (pre_params.method == auto_aim::BinaryMethod::ChannelSubtract)
    {
        const bool is_red = (pre_params.target_color == auto_aim::LightColor::Red);
        const std::string color_tag = is_red ? "red" : "blue";
        const int channel_threshold = is_red
            ? pre_params.channel_sub_threshold_red
            : pre_params.channel_sub_threshold_blue;
        method_tag = "ch" + std::to_string(channel_threshold) + "_" + color_tag;
    }
    else
    {
        method_tag = "gray" + std::to_string(pre_params.binary_threshold);
    }
    const std::string output_dir = output_root + "/test_lightbar_" + method_tag;

    //列出输入目录里的所有 png 图片
    std::vector<cv::String> files;
    cv::glob(input_dir + "/*.png", files, false);
    if (files.empty())
    {
        std::cerr << "no .png images in " << input_dir << '\n';
        return 1;
    }

    //创建本次输出文件夹
    std::filesystem::create_directories(output_dir);
    std::cout << "input=" << input_dir << " images=" << files.size()
              << " output=" << output_dir << '\n';

    for (const auto& file : files)
    {
        //读入一张图
        const cv::Mat frame = cv::imread(file);
        if (frame.empty())
        {
            std::cerr << "skip unreadable " << file << '\n';
            continue;
        }

        //跑预处理拿候选轮廓，再跑一次生产筛选用于通过数交叉核对
        const auto_aim::PreprocessResult pre = auto_aim::preprocess(frame, pre_params);
        const std::vector<auto_aim::LightBar> bars = auto_aim::filterLightBars(frame, pre, filter_params);

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //存一张二值图，方便对照候选是怎么来的
        cv::imwrite(output_dir + "/" + stem + "_binary.png", pre.binary);

        //遍历全部候选，按 filterLightBars 的公式重算四项指标并标注
        cv::Mat vis = frame.clone();
        int test_passed = 0;
        for (const auto& candidate : pre.candidates)
        {
            const cv::RotatedRect& rect = candidate.rect;
            const double area = candidate.area;

            //取最小外接矩形的长边短边，退化轮廓跳过（与生产一致）
            const double long_side = std::max(rect.size.width, rect.size.height);
            const double short_side = std::min(rect.size.width, rect.size.height);
            if (short_side <= kEpsilon)
            {
                continue;
            }
            const double rect_area = long_side * short_side;
            if (rect_area <= kEpsilon)
            {
                continue;
            }

            //长宽比
            const double aspect_ratio = long_side / short_side;

            //中心线与竖直方向的夹角（度），方向向量模为 0 时记 90 度
            const double vx = candidate.center_line[0];
            const double vy = candidate.center_line[1];
            const double norm = std::hypot(vx, vy);
            double line_angle_deg = 90.0;
            if (norm > kEpsilon)
            {
                line_angle_deg = std::acos(std::clamp(std::abs(vy) / norm, 0.0, 1.0)) * 180.0 / CV_PI;
            }

            //轮廓面积占外接矩形的比例
            const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

            //逐项判断是否在范围内
            const bool area_ok = area >= filter_params.min_area && area <= filter_params.max_area;
            const bool aspect_ok = aspect_ratio >= filter_params.min_aspect_ratio && aspect_ratio <= filter_params.max_aspect_ratio;
            const bool angle_ok = line_angle_deg >= filter_params.min_line_angle_deg && line_angle_deg <= filter_params.max_line_angle_deg;
            const bool fill_ok = fill_ratio >= filter_params.min_fill_ratio && fill_ratio <= filter_params.max_fill_ratio;
            const bool pass = area_ok && aspect_ok && angle_ok && fill_ok;
            if (pass)
            {
                ++test_passed;
            }

            //在范围绿、超范围红
            const cv::Scalar green(0, 255, 0);
            const cv::Scalar red(0, 0, 255);

            //画最小外接矩形，整体通过绿、被拒红
            cv::Point2f corners[4];
            rect.points(corners);
            for (int i = 0; i < 4; ++i)
            {
                cv::line(vis, corners[i], corners[(i + 1) % 4], pass ? green : red, 1);
            }

            //在轮廓旁边分四行标注四项值，每行按该项是否在范围单独着色
            const cv::Point anchor(cvRound(rect.center.x), cvRound(rect.center.y));
            cv::putText(vis, "A=" + fmt(area, 0), anchor + cv::Point(0, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, area_ok ? green : red, 1);
            cv::putText(vis, "AR=" + fmt(aspect_ratio, 2), anchor + cv::Point(0, 14),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, aspect_ok ? green : red, 1);
            cv::putText(vis, "ang=" + fmt(line_angle_deg, 1), anchor + cv::Point(0, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, angle_ok ? green : red, 1);
            cv::putText(vis, "fill=" + fmt(fill_ratio, 2), anchor + cv::Point(0, 42),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, fill_ok ? green : red, 1);
        }
        cv::imwrite(output_dir + "/" + stem + "_lightbar.png", vis);

        //filter_passed 与 test_passed 应一致，是对 filterLightBars 的一致性自检
        std::cout << stem
                  << " candidates=" << pre.candidates.size()
                  << " filter_passed=" << bars.size()
                  << " test_passed=" << test_passed << '\n';
    }

    std::cout << "done, results in " << output_dir << '\n';
    return 0;
}
