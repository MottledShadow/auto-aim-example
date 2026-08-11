#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "debug_draw.hpp"
#include "detector.hpp"

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录、标定文件路径（都可用命令行覆盖）
    const std::string input_dir = (argc > 1) ? argv[1] : "captures";
    const std::string output_root = (argc > 2) ? argv[2] : "test_output";
    const std::string calib_path = (argc > 3) ? argv[3] : "config/camera_calibration.yml";

    //各阶段参数走头文件默认值；PnP 参数（装甲尺寸/解算方法）同理
    auto_aim::PreprocessParams pre_params;
    auto_aim::LightBarFilterParams filter_params;
    auto_aim::LightBarMatcherParams matcher_params;
    auto_aim::PnpSolverParams pnp_params;

    //先加载相机标定，失败只告警不退出（后续 solvePnp 会返回空位姿）
    const auto_aim::CameraCalibration calibration = auto_aim::loadCalibration(calib_path);
    if (!calibration.error.empty())
    {
        std::cerr << "calibration load failed: " << calibration.error << '\n';
    }

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
    const std::string output_dir = output_root + "/test_pnp_" + method_tag;

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

        //预处理 → 灯条筛选 → 灯条配对 → PnP 位姿解算
        const auto_aim::PreprocessResult pre = auto_aim::preprocess(frame, pre_params);
        const std::vector<auto_aim::LightBar> bars = auto_aim::filterLightBars(frame, pre, filter_params);
        const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);
        const std::vector<auto_aim::ArmorPose> poses = auto_aim::solvePnp(armors, calibration, pnp_params);

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //逐个位姿画装甲四边形并标注到装甲板的距离(mm)，记下首个距离用于打印
        cv::Mat vis = frame.clone();
        double first_distance = 0.0;
        for (std::size_t p = 0; p < poses.size(); ++p)
        {
            const auto_aim::ArmorPose& pose = poses[p];
            const std::vector<cv::Point> quad = {
                pose.armor.left_light.top,
                pose.armor.right_light.top,
                pose.armor.right_light.bottom,
                pose.armor.left_light.bottom,
            };
            cv::polylines(vis, quad, true, cv::Scalar(0, 255, 0), 2);

            //tvec 的模即相机到装甲板中心的距离(mm)，记首个用于打印
            const double distance = cv::norm(pose.tvec);
            if (p == 0)
            {
                first_distance = distance;
            }

            //tvec/rvec 都是 3x1 的 CV_64F，把三个分量拼成两行文本标到装甲板旁
            char tvec_text[64];
            std::snprintf(tvec_text, sizeof(tvec_text), "t=[%.0f %.0f %.0f]mm",
                          pose.tvec.at<double>(0), pose.tvec.at<double>(1), pose.tvec.at<double>(2));
            char rvec_text[64];
            std::snprintf(rvec_text, sizeof(rvec_text), "r=[%.2f %.2f %.2f]",
                          pose.rvec.at<double>(0), pose.rvec.at<double>(1), pose.rvec.at<double>(2));

            //以装甲板中心为基准，rvec 在上一行、tvec 在下一行，避免重叠
            const cv::Point rvec_org(pose.armor.center.x, pose.armor.center.y - 10);
            const cv::Point tvec_org(pose.armor.center.x, pose.armor.center.y + 15);
            cv::putText(vis, rvec_text, rvec_org, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
            cv::putText(vis, tvec_text, tvec_org, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
        }
        cv::imwrite(output_dir + "/" + stem + "_pnp.png", vis);

        //打印每张图的各阶段计数与首个距离，solved 应与 matched 一致
        std::cout << stem
                  << " lightbars=" << bars.size()
                  << " matched=" << armors.size()
                  << " solved=" << poses.size()
                  << " first_distance_mm=" << static_cast<int>(first_distance) << '\n';
    }

    std::cout << "done, results in " << output_dir << '\n';
    return 0;
}
