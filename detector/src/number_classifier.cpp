#include "number_classifier.hpp"

#include <algorithm>
#include <fstream>

#include <opencv2/imgproc.hpp>

namespace auto_aim::detector
{

NumberClassifier::NumberClassifier(const NumberClassifierParams& params)
    : params_(params)
{
    net_ = cv::dnn::readNetFromONNX(params_.modelPath);
    if (net_.empty())
    {
        error_ = "cannot load onnx model: " + params_.modelPath;
        return;
    }

    std::ifstream labelFile(params_.labelPath);
    if (!labelFile.is_open())
    {
        error_ = "cannot open label file: " + params_.labelPath;
        return;
    }
    std::string line;
    while (std::getline(labelFile, line))
    {
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

    const int topY = (params_.warpHeight - params_.lightLength) / 2;
    const int bottomY = topY + params_.lightLength;

    const std::vector<cv::Point2f> src = {
        armor.leftLight.top,
        armor.rightLight.top,
        armor.rightLight.bottom,
        armor.leftLight.bottom,
    };

    const int warpWidth = (armor.type == ArmorType::Large)
        ? params_.largeArmorWidth
        : params_.smallArmorWidth;

    const std::vector<cv::Point2f> dst = {
        {0.0F, static_cast<float>(topY)},
        {static_cast<float>(warpWidth), static_cast<float>(topY)},
        {static_cast<float>(warpWidth), static_cast<float>(bottomY)},
        {0.0F, static_cast<float>(bottomY)},
    };

    const cv::Mat perspective = cv::getPerspectiveTransform(src, dst);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, perspective,
                        cv::Size(warpWidth, params_.warpHeight));

    const int roiX = (warpWidth - params_.roiWidth) / 2;
    diagnosis.roi = warped(cv::Rect(roiX, 0, params_.roiWidth, params_.warpHeight));

    cv::Mat gray;
    cv::cvtColor(diagnosis.roi, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, diagnosis.binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat blob;
    cv::dnn::blobFromImage(diagnosis.binary, blob, 1.0 / 255.0);

    net_.setInput(blob);
    cv::Mat logits = net_.forward();

    double maxLogit = 0.0;
    cv::minMaxLoc(logits, nullptr, &maxLogit);
    cv::Mat prob;
    cv::exp(logits - maxLogit, prob);
    prob /= cv::sum(prob)[0];

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
        const Diagnosis diagnosis = diagnose(frame, armor);
        const std::string number = labels_[diagnosis.classIndex];
        const float confidence = diagnosis.confidence;

        if (confidence < params_.confidenceThreshold)
        {
            continue;
        }

        if (number != "1" && number != "3" && number != "guard")
        {
            continue;
        }

        if (armor.type == ArmorType::Large && number != "1")
        {
            continue;
        }
        if (armor.type == ArmorType::Small && number == "1")
        {
            continue;
        }

        Armor kept = armor;
        kept.number = number;
        kept.confidence = confidence;
        result.push_back(kept);
    }

    return result;
}

}
