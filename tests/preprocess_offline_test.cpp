#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "debug_draw.hpp"
#include "detector.hpp"

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录（可用命令行覆盖，默认对应部署脚本的约定）
    const std::string input_dir = (argc > 1) ? argv[1] : "captures";
    const std::string output_root = (argc > 2) ? argv[2] : "test_output";

    //两套参数：灰度法 + 通道相减法（默认 ChannelSubtract + Red），每张图各跑一次
    auto_aim::PreprocessParams gray_params;
    gray_params.method = auto_aim::BinaryMethod::Gray;
    auto_aim::PreprocessParams channel_params;

    //文件夹名带上关键参数（灰度阈值 + 通道相减阈值 + 目标颜色），比时间戳直观
    const bool channel_is_red = (channel_params.target_color == auto_aim::LightColor::Red);
    const std::string color_tag = channel_is_red ? "red" : "blue";
    const int channel_threshold = channel_is_red
        ? channel_params.channel_sub_threshold_red
        : channel_params.channel_sub_threshold_blue;
    const std::string output_dir = output_root + "/test_gray"
        + std::to_string(gray_params.binary_threshold)
        + "_ch" + std::to_string(channel_threshold)
        + "_" + color_tag;

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

        //跑 detector 的预处理：灰度法和通道相减法各一次
        const auto_aim::PreprocessResult gray_result = auto_aim::preprocess(frame, gray_params);
        const auto_aim::PreprocessResult channel_result = auto_aim::preprocess(frame, channel_params);

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //两种方法的二值图各存一张
        cv::imwrite(output_dir + "/" + stem + "_binary_gray.png", gray_result.binary);
        cv::imwrite(output_dir + "/" + stem + "_binary_channel.png", channel_result.binary);

        //在原图上画出候选轮廓、最小外接矩形、中心线和面积数字
        cv::Mat vis = frame.clone();
        auto_aim::drawCandidates(vis, channel_result.candidates);
        cv::imwrite(output_dir + "/" + stem + "_vis_channel.png", vis);

        vis = frame.clone();
        auto_aim::drawCandidates(vis, gray_result.candidates);
        cv::imwrite(output_dir + "/" + stem + "_vis_binary.png", vis);

        std::cout << stem
                  << " gray_candidates=" << gray_result.candidates.size()
                  << " channel_candidates=" << channel_result.candidates.size() << '\n';
    }

    std::cout << "done, results in " << output_dir << '\n';
    return 0;
}
