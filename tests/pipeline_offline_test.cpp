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
    //第一个参数选调试阶段：preprocess | lightbar | armor | pnp，逐阶段离线跑同一批照片
    const std::string stage = (argc > 1) ? argv[1] : "";
    if (stage != "preprocess" && stage != "lightbar" && stage != "armor" && stage != "pnp")
    {
        std::cerr << "usage: pipeline_offline_test <preprocess|lightbar|armor|pnp>"
                     " [input_dir] [output_root] [calib]\n";
        return 1;
    }

    //输入图片目录、输出根目录、标定文件路径（都可用命令行覆盖，默认对应部署脚本的约定）
    const std::string input_dir = (argc > 2) ? argv[2] : "captures";
    const std::string output_root = (argc > 3) ? argv[3] : "test_output";
    const std::string calib_path = (argc > 4) ? argv[4] : "config/camera_calibration.yml";

    //各阶段参数都走头文件默认值；要改阈值直接改 detector.hpp
    auto_aim::PreprocessParams pre_params;
    auto_aim::LightBarFilterParams filter_params;
    auto_aim::LightBarMatcherParams matcher_params;
    auto_aim::PnpSolverParams pnp_params;

    //只有 pnp 阶段需要相机标定，失败只告警不退出（solvePnp 会返回空位姿）
    auto_aim::CameraCalibration calibration;
    if (stage == "pnp")
    {
        calibration = auto_aim::loadCalibration(calib_path);
        if (!calibration.error.empty())
        {
            std::cerr << "calibration load failed: " << calibration.error << '\n';
        }
    }

    //输出文件夹名标明灰度阈值，比时间戳直观
    const std::string method_tag = "gray" + std::to_string(pre_params.binary_threshold);

    std::string output_dir;
    if (stage == "preprocess")
    {
        output_dir = output_root + "/test_" + method_tag;
    }
    else
    {
        output_dir = output_root + "/test_" + stage + "_" + method_tag;
    }

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
    std::cout << "stage=" << stage << " input=" << input_dir << " images=" << files.size()
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

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //preprocess：灰度二值化跑一次，二值图 + 候选可视化各存一张
        if (stage == "preprocess")
        {
            const auto_aim::PreprocessResult gray_result = auto_aim::preprocess(frame, pre_params);

            //灰度二值图存一张
            cv::imwrite(output_dir + "/" + stem + "_binary.png", gray_result.binary);

            //在原图上画出候选轮廓、最小外接矩形、中心线和面积数字
            cv::Mat vis = frame.clone();
            auto_aim::drawCandidates(vis, gray_result.candidates);
            cv::imwrite(output_dir + "/" + stem + "_vis.png", vis);

            std::cout << stem
                      << " candidates=" << gray_result.candidates.size() << '\n';
            continue;
        }

        //lightbar/armor/pnp 都从预处理起步，先存一张二值图，方便对照灯条是怎么来的
        const auto_aim::PreprocessResult pre = auto_aim::preprocess(frame, pre_params);
        cv::imwrite(output_dir + "/" + stem + "_binary.png", pre.binary);

        //lightbar：预处理 → 筛选，遍历全部候选按公式重算四项指标标注
        if (stage == "lightbar")
        {
            const std::vector<auto_aim::LightBar> bars = auto_aim::filterLightBars(frame, pre, filter_params);

            cv::Mat vis = frame.clone();
            const std::size_t test_passed = auto_aim::drawLightBarMetrics(vis, pre.candidates, filter_params);
            cv::imwrite(output_dir + "/" + stem + "_lightbar.png", vis);

            //filter_passed 与 test_passed 应一致，是对 filterLightBars 的一致性自检
            std::cout << stem
                      << " candidates=" << pre.candidates.size()
                      << " filter_passed=" << bars.size()
                      << " test_passed=" << test_passed << '\n';
            continue;
        }

        //armor/pnp 都要先筛选灯条再配对
        const std::vector<auto_aim::LightBar> bars = auto_aim::filterLightBars(frame, pre, filter_params);
        const std::vector<auto_aim::Armor> armors = auto_aim::matchArmors(bars, matcher_params);

        //armor：遍历全部同色灯条对，按公式重算配对指标画装甲框+标注
        if (stage == "armor")
        {
            cv::Mat vis = frame.clone();
            const std::size_t draw_passed = auto_aim::drawArmorMetrics(vis, bars, matcher_params);
            cv::imwrite(output_dir + "/" + stem + "_armor.png", vis);

            //matched 与 draw_passed 应一致，是对 matchArmors 的一致性自检
            std::cout << stem
                      << " candidates=" << pre.candidates.size()
                      << " lightbars=" << bars.size()
                      << " matched=" << armors.size()
                      << " draw_passed=" << draw_passed << '\n';
            continue;
        }

        //pnp：配对后解算位姿，逐个位姿画装甲四边形并标注 rvec/tvec，记下首个距离用于打印
        const std::vector<auto_aim::ArmorPose> poses = auto_aim::solvePnp(armors, calibration, pnp_params);

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
