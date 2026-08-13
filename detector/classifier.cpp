#include "classifier.hpp"

#include <algorithm>
#include <fstream>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

NumberClassifier loadClassifier(const NumberClassifierParams& params)
{
    NumberClassifier classifier;

    //读 ONNX 网络，读不到就把原因记进 error
    classifier.net = cv::dnn::readNetFromONNX(params.model_path);
    if (classifier.net.empty())
    {
        classifier.error = "cannot load onnx model: " + params.model_path;
        return classifier;
    }

    //逐行读标签表，一行一个标签
    std::ifstream label_file(params.label_path);
    if (!label_file.is_open())
    {
        classifier.error = "cannot open label file: " + params.label_path;
        return classifier;
    }
    std::string line;
    while (std::getline(label_file, line))
    {
        //去掉行尾可能的回车（Windows 换行）
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        classifier.labels.push_back(line);
    }

    return classifier;
}

std::vector<Armor> classifyArmors(
    const cv::Mat& frame,
    const std::vector<Armor>& armors,
    NumberClassifier& classifier,
    const NumberClassifierParams& params)
{
    std::vector<Armor> result;

    //灯条落在矫正图中间 light_length 像素：top/bottom 上下对称留白
    const int top_y = (params.warp_height - params.light_length) / 2;
    const int bottom_y = top_y + params.light_length;

    for (const Armor& armor : armors)
    {
        //四端点源点：左上→右上→右下→左下，与 PnP 契约一致
        const std::vector<cv::Point2f> src = {
            armor.left_light.top,
            armor.right_light.top,
            armor.right_light.bottom,
            armor.left_light.bottom,
        };

        //矫正宽度按大小装甲取值：大装甲 54，小装甲 32
        const int warp_width = (armor.type == ArmorType::Large)
            ? params.large_armor_width
            : params.small_armor_width;

        //目标点：灯条端点落在中间列的 top_y/bottom_y 行
        const std::vector<cv::Point2f> dst = {
            {0.0F, static_cast<float>(top_y)},
            {static_cast<float>(warp_width), static_cast<float>(top_y)},
            {static_cast<float>(warp_width), static_cast<float>(bottom_y)},
            {0.0F, static_cast<float>(bottom_y)},
        };

        //透视矫正到 warp_width × warp_height
        const cv::Mat perspective = cv::getPerspectiveTransform(src, dst);
        cv::Mat warped;
        cv::warpPerspective(frame, warped, perspective,
                            cv::Size(warp_width, params.warp_height));

        //取中间 roi_width × warp_height 的 ROI
        const int roi_x = (warp_width - params.roi_width) / 2;
        const cv::Mat roi = warped(cv::Rect(roi_x, 0, params.roi_width, params.warp_height));

        //转灰度后大津法二值化
        cv::Mat gray;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binary;
        cv::threshold(gray, binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

        //归一化到 [0,1] 并做成 blob
        cv::Mat blob;
        cv::dnn::blobFromImage(binary, blob, 1.0 / 255.0);

        //推理得到 logits（1×类别数）
        classifier.net.setInput(blob);
        cv::Mat logits = classifier.net.forward();

        //softmax：减最大值防溢出，再指数归一化
        double max_logit = 0.0;
        cv::minMaxLoc(logits, nullptr, &max_logit);
        cv::Mat prob;
        cv::exp(logits - max_logit, prob);
        prob /= cv::sum(prob)[0];

        //取最大概率及其类别下标
        double confidence = 0.0;
        cv::Point class_point;
        cv::minMaxLoc(prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_point);
        const std::string number = classifier.labels[class_point.x];

        //过滤规则一：最大概率低于阈值丢弃
        if (confidence < params.confidence_threshold)
        {
            continue;
        }

        //过滤规则二：RMUL 只有英雄(1)/步兵(3)/哨兵(guard)，其余丢弃
        if (number != "1" && number != "3" && number != "guard")
        {
            continue;
        }

        //过滤规则三：大装甲只能是英雄(1)，大小与数字矛盾则丢弃
        if (armor.type == ArmorType::Large && number != "1")
        {
            continue;
        }
        if (armor.type == ArmorType::Small && number == "1")
        {
            continue;
        }

        //保留：写回数字与置信度
        Armor kept = armor;
        kept.number = number;
        kept.confidence = static_cast<float>(confidence);
        result.push_back(kept);
    }

    return result;
}

} // namespace auto_aim
