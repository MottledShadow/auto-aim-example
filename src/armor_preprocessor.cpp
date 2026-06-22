#include "vision_pipeline.hpp"

#include <cmath>

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

void applyMorphology(cv::Mat& image, int op, int kernel_size, int iterations)
{
    if (kernel_size <= 0 || iterations <= 0)
    {
        return;
    }
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image, image, op, kernel, cv::Point(-1, -1), iterations);
}

} // namespace

ArmorPreprocessResult preprocessFrame(const cv::Mat& frame, const ArmorPreprocessParams& params)
{
    ArmorPreprocessResult result;
    const cv::Mat gray = toGray(frame);

    cv::threshold(
        gray,
        result.binary,
        params.binary_threshold,
        255,
        cv::THRESH_BINARY);

    applyMorphology(result.binary, cv::MORPH_OPEN, params.open_kernel_size, params.morph_iterations);
    applyMorphology(result.binary, cv::MORPH_CLOSE, params.close_kernel_size, params.morph_iterations);

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
