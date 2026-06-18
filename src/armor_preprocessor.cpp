#include "armor_preprocessor.hpp"

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

cv::Mat toGray(const cv::Mat& frame)
{
    if (frame.channels() == 1)
    {
        return frame.clone();
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

} // namespace

ArmorPreprocessor::ArmorPreprocessor(ArmorPreprocessParams params) : params_(params)
{
}

ArmorPreprocessResult ArmorPreprocessor::process(const cv::Mat& frame) const
{
    ArmorPreprocessResult result;
    const cv::Mat gray = toGray(frame);

    cv::threshold(
        gray,
        result.binary,
        params_.binary_threshold,
        255,
        cv::THRESH_BINARY);

    if (params_.open_kernel_size > 0 && params_.morph_iterations > 0)
    {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(params_.open_kernel_size, params_.open_kernel_size));
        cv::morphologyEx(
            result.binary,
            result.binary,
            cv::MORPH_OPEN,
            kernel,
            cv::Point(-1, -1),
            params_.morph_iterations);
    }
    if (params_.close_kernel_size > 0 && params_.morph_iterations > 0)
    {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(params_.close_kernel_size, params_.close_kernel_size));
        cv::morphologyEx(
            result.binary,
            result.binary,
            cv::MORPH_CLOSE,
            kernel,
            cv::Point(-1, -1),
            params_.morph_iterations);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contour_input = result.binary.clone();
    cv::findContours(
        contour_input,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    result.candidates.reserve(contours.size());
    for (const auto& contour : contours)
    {
        if (contour.size() >= 2)
        {
            cv::Vec4f center_line;
            cv::fitLine(contour, center_line, cv::DIST_L2, 0, 0.01, 0.01);
            result.candidates.push_back(ContourCandidate{
                contour,
                cv::minAreaRect(contour),
                center_line,
                std::abs(cv::contourArea(contour)),
            });
        }
    }

    return result;
}

} // namespace auto_aim
