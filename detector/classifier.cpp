#include "classifier.hpp"

#include <algorithm>
#include <fstream>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

NumberClassifier::NumberClassifier(const NumberClassifierParams& params)
    : params_(params)
{
    //读 ONNX 网络，读不到就把原因记进 error_
    net_ = cv::dnn::readNetFromONNX(params_.model_path);
    if (net_.empty())
    {
        error_ = "cannot load onnx model: " + params_.model_path;
        return;
    }

    //逐行读标签表，一行一个标签
    std::ifstream label_file(params_.label_path);
    if (!label_file.is_open())
    {
        error_ = "cannot open label file: " + params_.label_path;
        return;
    }
    std::string line;
    while (std::getline(label_file, line))
    {
        //去掉行尾可能的回车（Windows 换行）
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        labels_.push_back(line);
    }
}

NumberClassifier::Diagnosis NumberClassifier::diagnose(const cv::Mat& frame, const Armor& armor)
{
    Diagnosis diagnosis;

    //灯条落在矫正图中间 light_length 像素：top/bottom 上下对称留白
    const int top_y = (params_.warp_height - params_.light_length) / 2;
    const int bottom_y = top_y + params_.light_length;

    //四端点源点：左上→右上→右下→左下，与 PnP 契约一致
    const std::vector<cv::Point2f> src = {
        armor.left_light.top,
        armor.right_light.top,
        armor.right_light.bottom,
        armor.left_light.bottom,
    };

    //矫正宽度按大小装甲取值：大装甲 54，小装甲 32
    const int warp_width = (armor.type == ArmorType::Large)
        ? params_.large_armor_width
        : params_.small_armor_width;

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
                        cv::Size(warp_width, params_.warp_height));

    //取中间 roi_width × warp_height 的 ROI
    const int roi_x = (warp_width - params_.roi_width) / 2;
    diagnosis.roi = warped(cv::Rect(roi_x, 0, params_.roi_width, params_.warp_height));

    //转灰度后大津法二值化
    cv::Mat gray;
    cv::cvtColor(diagnosis.roi, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, diagnosis.binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

    //归一化到 [0,1] 并做成 blob
    cv::Mat blob;
    cv::dnn::blobFromImage(diagnosis.binary, blob, 1.0 / 255.0);

    //推理得到 logits（1×类别数）
    net_.setInput(blob);
    cv::Mat logits = net_.forward();

    //softmax：减最大值防溢出，再指数归一化
    double max_logit = 0.0;
    cv::minMaxLoc(logits, nullptr, &max_logit);
    cv::Mat prob;
    cv::exp(logits - max_logit, prob);
    prob /= cv::sum(prob)[0];

    //取最大概率及其类别下标，并把全类别分布存进 probs
    double confidence = 0.0;
    cv::Point class_point;
    cv::minMaxLoc(prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_point);
    prob.reshape(1, 1).copyTo(diagnosis.probs);
    diagnosis.class_index = class_point.x;
    diagnosis.confidence = static_cast<float>(confidence);

    return diagnosis;
}

std::vector<Armor> NumberClassifier::classify(const cv::Mat& frame, const std::vector<Armor>& armors)
{
    std::vector<Armor> result;

    for (const Armor& armor : armors)
    {
        //推理拿到最大类下标与置信度
        const Diagnosis diagnosis = diagnose(frame, armor);
        const std::string number = labels_[diagnosis.class_index];
        const float confidence = diagnosis.confidence;

        //过滤规则一：最大概率低于阈值丢弃
        if (confidence < params_.confidence_threshold)
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
        kept.confidence = confidence;
        result.push_back(kept);
    }

    return result;
}

} // namespace auto_aim
