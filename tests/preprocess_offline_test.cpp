#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "detector.hpp"

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录（可用命令行覆盖，默认对应部署脚本的约定）
    const std::string input_dir = (argc > 1) ? argv[1] : "captures";
    const std::string output_root = (argc > 2) ? argv[2] : "test_output";

    //用当前时间戳生成本次测试的文件夹名，方便区分是哪一次做的测试
    const std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    const std::string output_dir = output_root + "/test_" + stamp;

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

    auto_aim::PreprocessParams params;
    for (const auto& file : files)
    {
        //读入一张图
        const cv::Mat frame = cv::imread(file);
        if (frame.empty())
        {
            std::cerr << "skip unreadable " << file << '\n';
            continue;
        }

        //跑 detector 的预处理
        const auto_aim::ArmorPreprocessResult result = auto_aim::preprocess(frame, params);

        //取文件名（不含目录和扩展名）作为输出前缀
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);

        //存二值图
        cv::imwrite(output_dir + "/" + stem + "_binary.png", result.binary);

        //在原图上画出候选轮廓、最小外接矩形、中心线和面积数字
        cv::Mat vis = frame.clone();
        for (const auto& candidate : result.candidates)
        {
            //候选轮廓（绿色）
            const std::vector<std::vector<cv::Point>> one_contour{candidate.contour};
            cv::drawContours(vis, one_contour, -1, cv::Scalar(0, 255, 0), 1);

            //最小外接矩形（黄色）
            cv::Point2f corners[4];
            candidate.rect.points(corners);
            for (int i = 0; i < 4; ++i)
            {
                cv::line(vis, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
            }

            //fitLine 中心线（红色），沿方向向量往两边延伸
            const float vx = candidate.center_line[0];
            const float vy = candidate.center_line[1];
            const float x0 = candidate.center_line[2];
            const float y0 = candidate.center_line[3];
            const float length = 30.0F;
            const cv::Point p1(cvRound(x0 - vx * length), cvRound(y0 - vy * length));
            const cv::Point p2(cvRound(x0 + vx * length), cvRound(y0 + vy * length));
            cv::line(vis, p1, p2, cv::Scalar(0, 0, 255), 1);

            //面积数字
            cv::putText(
                vis,
                "A=" + std::to_string(static_cast<int>(candidate.area)),
                candidate.rect.center,
                cv::FONT_HERSHEY_SIMPLEX,
                0.4,
                cv::Scalar(0, 0, 255),
                1);
        }
        cv::imwrite(output_dir + "/" + stem + "_vis.png", vis);

        std::cout << stem << " candidates=" << result.candidates.size() << '\n';
    }

    std::cout << "done, results in " << output_dir << '\n';
    return 0;
}
