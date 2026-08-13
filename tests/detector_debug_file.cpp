#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "debug_draw.hpp"
#include "geometry_detector.hpp"
#include "number_classifier.hpp"
#include "pnp_solver.hpp"

int main(int argc, char** argv)
{
    //输入图片目录、输出根目录、标定文件路径（都可用命令行覆盖，默认对应部署脚本的约定）
    const std::string inputDir = (argc > 1) ? argv[1] : "captures";
    const std::string outputRoot = (argc > 2) ? argv[2] : "test_output";
    const std::string calibPath = (argc > 3) ? argv[3] : "config/camera_calibration.yml";

    //几何检测器无状态，参数走头文件默认值；要改阈值直接改 geometry_detector.hpp
    auto_aim::GeometryDetector detector;

    //数字分类器：构造时加载一次网络+标签（有状态、开销大），失败只告警
    auto_aim::NumberClassifierParams numberParams;
    auto_aim::NumberClassifier classifier(numberParams);
    if (!classifier.error().empty())
    {
        std::cerr << "classifier load failed: " << classifier.error() << '\n';
    }

    //PnP 求解器：需要相机标定，失败只告警（solve 会返回空位姿）
    const auto_aim::CameraCalibration calibration = auto_aim::loadCalibration(calibPath);
    if (!calibration.error.empty())
    {
        std::cerr << "calibration load failed: " << calibration.error << '\n';
    }
    const auto_aim::PnpSolver pnpSolver(calibration);

    //输出根目录下按灰度阈值分一层，便于对比不同阈值跑出来的整批结果
    const std::string methodTag = "gray" + std::to_string(detector.preprocessParams.binaryThreshold);
    const std::string runDir = outputRoot + "/" + methodTag;

    //列出输入目录里的所有 png 图片
    std::vector<cv::String> files;
    cv::glob(inputDir + "/*.png", files, false);
    if (files.empty())
    {
        std::cerr << "no .png images in " << inputDir << '\n';
        return 1;
    }
    std::filesystem::create_directories(runDir);
    std::cout << "input=" << inputDir << " images=" << files.size() << " output=" << runDir << '\n';

    for (const auto& file : files)
    {
        //读入一张图
        const cv::Mat frame = cv::imread(file);
        if (frame.empty())
        {
            std::cerr << "skip unreadable " << file << '\n';
            continue;
        }

        //取文件名（不含目录和扩展名）作为本图的输出子目录，五个阶段的图都落在里面
        const size_t slash = file.find_last_of("/\\");
        const size_t dot = file.find_last_of('.');
        const std::string stem = file.substr(slash + 1, dot - slash - 1);
        const std::string dir = runDir + "/" + stem;
        std::filesystem::create_directories(dir);

        //几何三阶段：预处理 → 灯条筛选 → 装甲配对（armors 是后面 PnP 的输入，与分类无关）
        const auto_aim::PreprocessResult pre = detector.preprocess(frame);
        const std::vector<auto_aim::LightBar> bars = detector.filterLightBars(frame, pre);
        const std::vector<auto_aim::Armor> armors = detector.matchArmors(bars);

        //1_preprocess：原图 | 二值图 左右对照，标灰度阈值，方便一眼看二值化效果
        const cv::Mat compare = auto_aim::sideBySide(frame, pre.binary, detector.preprocessParams.binaryThreshold);
        cv::imwrite(dir + "/1_preprocess.png", compare);

        //2_lightbar：逐候选画轮廓/最小外接矩形/中心线（各一色、细线、不标序号），左上角图例标四项指标
        cv::Mat lightbarVis = frame.clone();
        const std::size_t lightbarPassed = auto_aim::drawLightBarMetrics(lightbarVis, pre.candidates, detector.filterParams);
        cv::imwrite(dir + "/2_lightbar.png", lightbarVis);

        //3_armor：先给每根灯条标索引，再逐同色对画装甲框，左上角图例标 "i+j" 的五项配对判据
        cv::Mat armorVis = frame.clone();
        const std::size_t armorPassed = auto_aim::drawArmorMetrics(armorVis, bars, detector.matcherParams);
        cv::imwrite(dir + "/3_armor.png", armorVis);

        //4_number：逐块配对装甲板做数字诊断（不过滤），旁标 数字+置信度；KEEP 绿/REJECT 红（无论过没过筛选都标）
        //         同时存每块矫正 ROI/二值图，方便调分类器
        cv::Mat numberVis = frame.clone();
        std::size_t kept = 0;
        for (std::size_t a = 0; a < armors.size(); ++a)
        {
            const auto_aim::Armor& armor = armors[a];

            //diagnose 与 classify 同一套推理，但不做去留过滤，只拿数字/置信度做标注
            const auto_aim::NumberClassifier::Diagnosis d = classifier.diagnose(frame, armor);
            cv::imwrite(dir + "/number_roi_" + std::to_string(a) + ".png", d.roi);
            cv::imwrite(dir + "/number_bin_" + std::to_string(a) + ".png", d.binary);

            const double confidence = d.confidence;
            const std::string number = classifier.labels()[d.classIndex];

            //按 classify 的四条去留规则算 KEEP/REJECT，只决定标注颜色，不影响 PnP
            bool keep = true;
            if (confidence < numberParams.confidenceThreshold)
            {
                keep = false;
            }
            else if (number != "1" && number != "3" && number != "guard")
            {
                keep = false;
            }
            else if (armor.type == auto_aim::ArmorType::Large && number != "1")
            {
                keep = false;
            }
            else if (armor.type == auto_aim::ArmorType::Small && number == "1")
            {
                keep = false;
            }
            if (keep)
            {
                ++kept;
            }

            //画框：KEEP 绿、REJECT 红，标注 数字(置信度)
            const std::vector<cv::Point> quad = {
                armor.leftLight.top,
                armor.rightLight.top,
                armor.rightLight.bottom,
                armor.leftLight.bottom,
            };
            const cv::Scalar color = keep ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::polylines(numberVis, quad, true, color, 2);
            char label[32];
            std::snprintf(label, sizeof(label), "%s %.2f", number.c_str(), confidence);
            cv::putText(numberVis, label, cv::Point(armor.center.x, armor.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }
        cv::imwrite(dir + "/4_number.png", numberVis);

        //5_pnp：直接对全部配对装甲板解算位姿（绕开分类过滤，没贴贴纸也能看到位姿），逐块画框标 rvec/tvec
        const std::vector<auto_aim::ArmorPose> poses = pnpSolver.solve(armors);
        cv::Mat pnpVis = frame.clone();
        double firstDistance = 0.0;
        for (std::size_t p = 0; p < poses.size(); ++p)
        {
            const auto_aim::ArmorPose& pose = poses[p];
            const std::vector<cv::Point> quad = {
                pose.armor.leftLight.top,
                pose.armor.rightLight.top,
                pose.armor.rightLight.bottom,
                pose.armor.leftLight.bottom,
            };
            cv::polylines(pnpVis, quad, true, cv::Scalar(0, 255, 0), 2);

            //tvec 的模即相机到装甲板中心的距离(mm)，记首个用于打印
            const double distance = cv::norm(pose.tvec);
            if (p == 0)
            {
                firstDistance = distance;
            }

            //tvec/rvec 都是 3x1 的 CV_64F，把三个分量拼成两行文本标到装甲板旁
            char tvecText[64];
            std::snprintf(tvecText, sizeof(tvecText), "t=[%.0f %.0f %.0f]mm",
                          pose.tvec.at<double>(0), pose.tvec.at<double>(1), pose.tvec.at<double>(2));
            char rvecText[64];
            std::snprintf(rvecText, sizeof(rvecText), "r=[%.2f %.2f %.2f]",
                          pose.rvec.at<double>(0), pose.rvec.at<double>(1), pose.rvec.at<double>(2));

            //以装甲板中心为基准，rvec 在上一行、tvec 在下一行，避免重叠
            const cv::Point rvecOrg(pose.armor.center.x, pose.armor.center.y - 10);
            const cv::Point tvecOrg(pose.armor.center.x, pose.armor.center.y + 15);
            cv::putText(pnpVis, rvecText, rvecOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
            cv::putText(pnpVis, tvecText, tvecOrg, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
        }
        cv::imwrite(dir + "/5_pnp.png", pnpVis);

        //每张图一行汇总各阶段计数；lightbarPassed 应=lightbars、armorPassed 应=matched（对绘制/生产一致性的自检）
        std::cout << stem
                  << " candidates=" << pre.candidates.size()
                  << " lightbars=" << bars.size() << "(draw " << lightbarPassed << ")"
                  << " matched=" << armors.size() << "(draw " << armorPassed << ")"
                  << " kept=" << kept
                  << " solved=" << poses.size()
                  << " first_distance_mm=" << static_cast<int>(firstDistance) << '\n';
    }

    std::cout << "done, results in " << runDir << "/<image>/{1_preprocess..5_pnp}.png\n";
    return 0;
}
