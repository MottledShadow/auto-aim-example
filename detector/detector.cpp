#include "detector.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace auto_aim
{

ArmorPreprocessResult preprocess(const cv::Mat& frame, const PreprocessParams& params)
{
    ArmorPreprocessResult result;

    //转为灰度图
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    //二值化
    cv::threshold(
        gray,
        result.binary,
        params.binary_threshold,
        255,
        cv::THRESH_BINARY);

    //开闭运算降噪
    const cv::Mat open_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(params.open_kernel_size, params.open_kernel_size));
    cv::morphologyEx(
        result.binary, result.binary, cv::MORPH_OPEN, open_kernel, cv::Point(-1, -1), params.morph_iterations);
    const cv::Mat close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(params.close_kernel_size, params.close_kernel_size));
    cv::morphologyEx(
        result.binary, result.binary, cv::MORPH_CLOSE, close_kernel, cv::Point(-1, -1), params.morph_iterations);

    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contour_input = result.binary.clone();
    cv::findContours(
        contour_input,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    //计算轮廓的最小外接矩形和中心线并存储到结果中
    result.candidates.reserve(contours.size());
    for (const auto& contour : contours)
    {
        if (contour.size() >= 2)
        {
            cv::Vec4f center_line;
            cv::fitLine(contour, center_line, cv::DIST_L2, 0, 0.01, 0.01);  //后面需要考虑是否要往后面过程移动
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
