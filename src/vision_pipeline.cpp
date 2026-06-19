#include "vision_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_aim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;

float longSide(const cv::RotatedRect& rect)
{
    return std::max(rect.size.width, rect.size.height);
}

float shortSide(const cv::RotatedRect& rect)
{
    return std::min(rect.size.width, rect.size.height);
}

std::vector<cv::Point2f> armorCorners(const LightBar& left, const LightBar& right)
{
    return {
        left.top,
        right.top,
        right.bottom,
        left.bottom,
    };
}

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

double lineAngleFromVerticalDeg(const cv::Vec4f& line)
{
    const double vx = static_cast<double>(line[0]);
    const double vy = static_cast<double>(line[1]);
    const double norm = std::hypot(vx, vy);
    if (norm <= kEpsilon)
    {
        return 90.0;
    }

    const double cos_to_vertical = std::clamp(std::abs(vy) / norm, 0.0, 1.0);
    return std::acos(cos_to_vertical) * 180.0 / kPi;
}

bool inRange(double value, double min_value, double max_value)
{
    return value >= min_value && value <= max_value;
}

LightColor detectLightColor(const cv::Mat& frame, const std::vector<cv::Point>& contour)
{
    if (frame.channels() < 3)
    {
        return LightColor::Unknown;
    }

    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(255), cv::FILLED);

    const cv::Scalar mean = cv::mean(frame, mask);
    if (mean[2] > mean[0])
    {
        return LightColor::Red;
    }
    if (mean[0] > mean[2])
    {
        return LightColor::Blue;
    }
    return LightColor::Unknown;
}

LightBar makeLightBar(
    const cv::RotatedRect& rect,
    double line_angle_deg,
    LightColor color,
    double area)
{
    const float length = longSide(rect);
    const float width = shortSide(rect);
    const cv::Point2f center = rect.center;

    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f p1;
    cv::Point2f p2;

    const double edge01 = std::hypot(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y);
    const double edge12 = std::hypot(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y);
    if (edge01 <= edge12)
    {
        p1 = (vertices[0] + vertices[1]) * 0.5F;
        p2 = (vertices[2] + vertices[3]) * 0.5F;
    }
    else
    {
        p1 = (vertices[1] + vertices[2]) * 0.5F;
        p2 = (vertices[3] + vertices[0]) * 0.5F;
    }

    if (p1.y > p2.y)
    {
        std::swap(p1, p2);
    }

    return LightBar{
        color,
        p1,
        p2,
        center,
        length,
        width,
        static_cast<float>(line_angle_deg),
        static_cast<float>(area)};
}

bool sameKnownColor(const LightBar& a, const LightBar& b)
{
    return a.color != LightColor::Unknown && a.color == b.color;
}

bool containsPoint(const std::vector<cv::Point2f>& region, const cv::Point2f& point)
{
    return cv::pointPolygonTest(region, point, false) >= 0.0;
}

bool hasLightBetween(
    const std::vector<LightBar>& lights,
    std::size_t left_index,
    std::size_t right_index,
    const std::vector<cv::Point2f>& region)
{
    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        if (i == left_index || i == right_index)
        {
            continue;
        }

        const LightBar& light = lights[i];
        if (containsPoint(region, light.top) ||
            containsPoint(region, light.bottom) ||
            containsPoint(region, light.center))
        {
            return true;
        }
    }
    return false;
}

ArmorType classifyArmor(double center_distance_ratio, const ArmorMatcherParams& params)
{
    if (center_distance_ratio >= params.large_armor_min_center_distance_ratio)
    {
        return ArmorType::Large;
    }
    return ArmorType::Small;
}

Armor makeArmor(const LightBar& left, const LightBar& right, ArmorType type)
{
    const cv::Point2f center = (left.center + right.center) * 0.5F;
    return Armor{left, right, center, type};
}

cv::Mat normalizeCameraMatrix(const cv::Mat& matrix)
{
    cv::Mat converted;
    matrix.convertTo(converted, CV_64F);
    if (converted.rows != 3 || converted.cols != 3)
    {
        throw std::runtime_error("camera_matrix must be 3x3");
    }
    return converted;
}

cv::Mat normalizeDistCoeffs(const cv::Mat& coeffs)
{
    cv::Mat converted;
    coeffs.convertTo(converted, CV_64F);
    if (converted.rows != 1 && converted.cols != 1)
    {
        converted = converted.reshape(1, 1).clone();
    }
    return converted;
}

std::vector<cv::Point3f> makeObjectPoints(const Armor& armor, const PnpSolverParams& params)
{
    const bool is_large = armor.type == ArmorType::Large;
    const float width = static_cast<float>(is_large ? params.large_armor_width : params.small_armor_width);
    const float height = static_cast<float>(is_large ? params.large_armor_height : params.small_armor_height);
    const float half_width = width * 0.5F;
    const float half_height = height * 0.5F;

    return {
        {-half_width, -half_height, 0.0F},
        { half_width, -half_height, 0.0F},
        { half_width,  half_height, 0.0F},
        {-half_width,  half_height, 0.0F},
    };
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

LightBarFilterResult filterLightBars(
    const cv::Mat& frame,
    const ArmorPreprocessResult& preprocess,
    const LightBarFilterParams& params)
{
    LightBarFilterResult result;
    result.candidates.reserve(preprocess.candidates.size());

    for (const ContourCandidate& geom : preprocess.candidates)
    {
        const std::vector<cv::Point>& contour = geom.contour;
        const cv::RotatedRect& rect = geom.rect;
        const cv::Vec4f& line = geom.center_line;
        const double area = geom.area;
        const double long_side = longSide(rect);
        const double short_side = shortSide(rect);
        if (short_side <= kEpsilon)
        {
            continue;
        }

        const double rect_area = long_side * short_side;
        if (rect_area <= kEpsilon)
        {
            continue;
        }

        const double aspect_ratio = long_side / short_side;
        const double line_angle_deg = lineAngleFromVerticalDeg(line);
        const double fill_ratio = std::clamp(area / rect_area, 0.0, 1.0);

        if (!inRange(area, params.min_area, params.max_area) ||
            !inRange(aspect_ratio, params.min_aspect_ratio, params.max_aspect_ratio) ||
            !inRange(line_angle_deg, params.min_line_angle_deg, params.max_line_angle_deg) ||
            !inRange(fill_ratio, params.min_fill_ratio, params.max_fill_ratio))
        {
            continue;
        }

        const LightColor color = detectLightColor(frame, contour);
        result.candidates.emplace_back(makeLightBar(rect, line_angle_deg, color, area));
    }

    return result;
}

ArmorMatchResult matchArmors(const LightBarFilterResult& light_bars, const ArmorMatcherParams& params)
{
    ArmorMatchResult result;
    const std::vector<LightBar>& lights = light_bars.candidates;

    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        for (std::size_t j = i + 1; j < lights.size(); ++j)
        {
            std::size_t left_index = i;
            std::size_t right_index = j;
            if (lights[right_index].center.x < lights[left_index].center.x)
            {
                std::swap(left_index, right_index);
            }

            const LightBar& left = lights[left_index];
            const LightBar& right = lights[right_index];

            if (!sameKnownColor(left, right))
            {
                continue;
            }

            const double min_length = std::min(left.length, right.length);
            if (min_length <= kEpsilon)
            {
                continue;
            }

            const double length_ratio = std::max(left.length, right.length) / min_length;
            const double angle_diff = std::abs(left.angle - right.angle);
            const double center_y_diff = std::abs(left.center.y - right.center.y);
            const double average_height = (left.length + right.length) * 0.5;
            if (average_height <= kEpsilon)
            {
                continue;
            }

            const double center_distance_ratio =
                std::hypot(left.center.x - right.center.x, left.center.y - right.center.y) /
                average_height;

            if (length_ratio > params.max_light_length_ratio ||
                angle_diff > params.max_light_angle_diff_deg ||
                center_y_diff > params.max_light_center_y_diff ||
                center_distance_ratio < params.min_center_distance_ratio ||
                center_distance_ratio > params.max_center_distance_ratio)
            {
                continue;
            }

            const std::vector<cv::Point2f> region = armorCorners(left, right);
            if (hasLightBetween(lights, left_index, right_index, region))
            {
                continue;
            }

            result.candidates.emplace_back(
                makeArmor(left, right, classifyArmor(center_distance_ratio, params)));
        }
    }

    return result;
}

Calibration loadCalibration(const std::string& path)
{
    Calibration calibration;
    if (path.empty())
    {
        return calibration;
    }

    try
    {
        cv::FileStorage storage(path, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open calibration file: " + path);
        }

        storage["camera_matrix"] >> calibration.camera_matrix;
        storage["dist_coeffs"] >> calibration.dist_coeffs;
        if (calibration.camera_matrix.empty() || calibration.dist_coeffs.empty())
        {
            throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
        }

        calibration.camera_matrix = normalizeCameraMatrix(calibration.camera_matrix);
        calibration.dist_coeffs = normalizeDistCoeffs(calibration.dist_coeffs);
    }
    catch (const std::exception& ex)
    {
        calibration.camera_matrix.release();
        calibration.dist_coeffs.release();
        calibration.error = ex.what();
    }

    return calibration;
}

PnpSolveResult solvePnp(
    const ArmorMatchResult& armors,
    const Calibration& calibration,
    const PnpSolverParams& params)
{
    PnpSolveResult result;
    result.calibration_error = calibration.error;
    if (calibration.camera_matrix.empty())
    {
        return result;
    }

    for (const Armor& armor : armors.candidates)
    {
        const std::vector<cv::Point2f> image_points = armorCorners(armor.left_light, armor.right_light);
        const std::vector<cv::Point3f> object_points = makeObjectPoints(armor, params);

        cv::Mat rvec;
        cv::Mat tvec;
        const bool solved = cv::solvePnP(
            object_points,
            image_points,
            calibration.camera_matrix,
            calibration.dist_coeffs,
            rvec,
            tvec,
            false,
            params.solve_pnp_method);
        if (!solved)
        {
            continue;
        }

        ArmorPose pose;
        pose.armor = armor;
        pose.rvec = rvec;
        pose.tvec = tvec;
        result.poses.emplace_back(std::move(pose));
    }

    return result;
}

VisionPipelineResult runPipeline(const cv::Mat& frame, const Calibration& calibration)
{
    VisionPipelineResult result;
    result.preprocess = preprocessFrame(frame);
    result.light_bars = filterLightBars(frame, result.preprocess);
    result.armors = matchArmors(result.light_bars);
    result.pnp = solvePnp(result.armors, calibration);
    return result;
}

} // namespace auto_aim
