#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "debug_draw.hpp"
#include "detector.hpp"

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录（可用命令行覆盖，默认对应部署脚本的约定）
    const std::string input_dir = (argc > 1) ? argv[1] : "captures";
    const std::string output_root = (argc > 2) ? argv[2] : "test_output";

    //预处理参数（默认通道相减+蓝）、灯条筛选参数、待调的灯条配对参数，均走头文件默认值
    //要测灰度或红色，直接改这里的 pre_params.method / target_color
    auto_aim::PreprocessParams pre_params;
    auto_aim::LightBarFilterParams filter_params;
    auto_aim::LightBarMatcherParams matcher_params;

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
    const std::string output_dir = output_root + "/test_armor_" + method_tag;

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

        //预处理拿候选轮廓 → 灯条筛选 → 灯条配对，配对结果用于装甲数交叉核对
        const auto_aim::PreprocessResult pre = auto_aim::preprocess(frame, pre_params);
        const std::vector<auto_aim::LightBar> bars = auto_aim::filterLightBars(frame, pre, filter_params);
        const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //存一张二值图，方便对照灯条是怎么来的
        cv::imwrite(output_dir + "/" + stem + "_binary.png", pre.binary);

        //遍历全部同色灯条对，按 matchArmors 的公式重算指标画装甲框+五项标注，返回接受数
        cv::Mat vis = frame.clone();
        const std::size_t draw_passed = auto_aim::drawArmorMetrics(vis, bars, matcher_params);
        cv::imwrite(output_dir + "/" + stem + "_armor.png", vis);

        //matched 与 draw_passed 应一致，是对 matchArmors 的一致性自检
        std::cout << stem
                  << " candidates=" << pre.candidates.size()
                  << " lightbars=" << bars.size()
                  << " matched=" << armors.size()
                  << " draw_passed=" << draw_passed << '\n';
    }

    std::cout << "done, results in " << output_dir << '\n';
    return 0;
}
