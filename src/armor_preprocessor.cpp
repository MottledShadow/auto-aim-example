#include "armor_preprocessor.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

void validateParams(const ArmorPreprocessParams& params)
{
    if (params.binary_threshold < 0 || params.binary_threshold > 255)
    {
        throw std::invalid_argument("binary_threshold must be in [0, 255]");
    }
    if (params.open_kernel_size < 0 || params.close_kernel_size < 0)
    {
        throw std::invalid_argument("morphology kernel size must be non-negative");
    }
    if (params.morph_iterations < 0)
    {
        throw std::invalid_argument("morph_iterations must be non-negative");
    }
}

void applyMorphology(cv::Mat& image, int operation, int kernel_size, int iterations)
{
    if (kernel_size <= 0 || iterations <= 0)
    {
        return;
    }

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image, image, operation, kernel, cv::Point(-1, -1), iterations);
}

cv::Mat toGray(const cv::Mat& frame)
{
    if (frame.empty())
    {
        throw std::invalid_argument("frame is empty");
    }

    cv::Mat gray;
    if (frame.channels() == 1)
    {
        gray = frame.clone();
    }
    else if (frame.channels() == 3)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else if (frame.channels() == 4)
    {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    }
    else
    {
        throw std::invalid_argument("unsupported frame channel count");
    }
    return gray;
}

} // namespace

ArmorPreprocessor::ArmorPreprocessor(ArmorPreprocessParams params) : params_(params)
{
    validateParams(params_);
}

ArmorPreprocessResult ArmorPreprocessor::process(const cv::Mat& frame) const
{
    ArmorPreprocessResult result;
    result.gray = toGray(frame);

    cv::threshold(
        result.gray,
        result.binary,
        params_.binary_threshold,
        255,
        cv::THRESH_BINARY);

    applyMorphology(
        result.binary,
        cv::MORPH_OPEN,
        params_.open_kernel_size,
        params_.morph_iterations);
    applyMorphology(
        result.binary,
        cv::MORPH_CLOSE,
        params_.close_kernel_size,
        params_.morph_iterations);

    cv::Mat contour_input = result.binary.clone();
    cv::findContours(
        contour_input,
        result.contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    result.candidate_rects.reserve(result.contours.size());
    result.candidate_center_lines.reserve(result.contours.size());
    for (const auto& contour : result.contours)
    {
        if (contour.size() >= 2)
        {
            result.candidate_rects.emplace_back(cv::minAreaRect(contour));

            cv::Vec4f center_line;
            cv::fitLine(contour, center_line, cv::DIST_L2, 0, 0.01, 0.01);
            result.candidate_center_lines.emplace_back(center_line);
        }
    }

    return result;
}

const ArmorPreprocessParams& ArmorPreprocessor::params() const
{
    return params_;
}

} // namespace auto_aim
