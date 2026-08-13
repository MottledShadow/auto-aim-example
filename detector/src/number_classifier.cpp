#include "number_classifier.hpp"

#include <algorithm>
#include <fstream>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

NumberClassifier::NumberClassifier(const NumberClassifierParams& params)
    : params_(params)
{
    //读 ONNX 网络，读不到就把原因记进 error_
    net_ = cv::dnn::readNetFromONNX(params_.modelPath);
    if (net_.empty())
    {
        error_ = "cannot load onnx model: " + params_.modelPath;
        return;
    }

    //逐行读标签表，一行一个标签
    std::ifstream labelFile(params_.labelPath);
    if (!labelFile.is_open())
    {
        error_ = "cannot open label file: " + params_.labelPath;
        return;
    }
    std::string line;
    while (std::getline(labelFile, line))
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

    //灯条落在矫正图中间 lightLength 像素：top/bottom 上下对称留白
    const int topY = (params_.warpHeight - params_.lightLength) / 2;
    const int bottomY = topY + params_.lightLength;

    //四端点源点：左上→右上→右下→左下，与 PnP 契约一致
    const std::vector<cv::Point2f> src = {
        armor.leftLight.top,
        armor.rightLight.top,
        armor.rightLight.bottom,
        armor.leftLight.bottom,
    };

    //矫正宽度按大小装甲取值：大装甲 54，小装甲 32
    const int warpWidth = (armor.type == ArmorType::Large)
        ? params_.largeArmorWidth
        : params_.smallArmorWidth;

    //目标点：灯条端点落在中间列的 topY/bottomY 行
    const std::vector<cv::Point2f> dst = {
        {0.0F, static_cast<float>(topY)},
        {static_cast<float>(warpWidth), static_cast<float>(topY)},
        {static_cast<float>(warpWidth), static_cast<float>(bottomY)},
        {0.0F, static_cast<float>(bottomY)},
    };

    //透视矫正到 warpWidth × warpHeight
    const cv::Mat perspective = cv::getPerspectiveTransform(src, dst);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, perspective,
                        cv::Size(warpWidth, params_.warpHeight));

    //取中间 roiWidth × warpHeight 的 ROI
    const int roiX = (warpWidth - params_.roiWidth) / 2;
    diagnosis.roi = warped(cv::Rect(roiX, 0, params_.roiWidth, params_.warpHeight));

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
    double maxLogit = 0.0;
    cv::minMaxLoc(logits, nullptr, &maxLogit);
    cv::Mat prob;
    cv::exp(logits - maxLogit, prob);
    prob /= cv::sum(prob)[0];

    //取最大概率及其类别下标，并把全类别分布存进 probs
    double confidence = 0.0;
    cv::Point classPoint;
    cv::minMaxLoc(prob.reshape(1, 1), nullptr, &confidence, nullptr, &classPoint);
    prob.reshape(1, 1).copyTo(diagnosis.probs);
    diagnosis.classIndex = classPoint.x;
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
        const std::string number = labels_[diagnosis.classIndex];
        const float confidence = diagnosis.confidence;

        //过滤规则一：最大概率低于阈值丢弃
        if (confidence < params_.confidenceThreshold)
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
