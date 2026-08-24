#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "hik_camera.hpp"

namespace
{

constexpr int kTimeoutMs = 1000;
constexpr int kRequiredViews = 20;
constexpr double kSquareSize = 60.0;
constexpr double kPreviewScale = 0.25;
constexpr char kOutputPath[] = "config/camera_calibration.yml";
const cv::Size kPatternSize(11, 8);

void printUsage(const char* exe)
{
    std::cout
        << "Usage:\n"
        << "  " << exe << "\n"
        << "  " << exe << " --images DIR\n\n"
        << "Calibrate a camera with an 11x8-inner-corner chessboard using at least 20 views.\n"
        << "The square size is 60 mm and the result is written to " << kOutputPath << ".\n\n"
        << "Live keys: SPACE accept view, u undo last, ENTER calibrate, ESC quit\n";
}

std::vector<cv::Point3f> makeObjectPoints()
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(kPatternSize.area()));
    for (int row = 0; row < kPatternSize.height; ++row)
    {
        for (int col = 0; col < kPatternSize.width; ++col)
        {
            points.emplace_back(
                static_cast<float>(col * kSquareSize),
                static_cast<float>(row * kSquareSize),
                0.0F);
        }
    }
    return points;
}

cv::Mat toGray(const cv::Mat& image)
{
    if (image.channels() == 1)
    {
        return image;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

bool detectCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners, bool refine)
{
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!refine)
    {
        flags |= cv::CALIB_CB_FAST_CHECK;
    }
    if (!cv::findChessboardCorners(gray, kPatternSize, corners, flags))
    {
        return false;
    }
    if (refine)
    {
        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
    }
    return true;
}

int calibrateAndSave(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    cv::Size imageSize)
{
    if (imagePoints.size() < kRequiredViews)
    {
        std::cerr << "need at least " << kRequiredViews
                  << " detected views to calibrate, got " << imagePoints.size() << '\n';
        return 1;
    }

    const std::vector<cv::Point3f> object = makeObjectPoints();
    const std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), object);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    const double rms = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,
        cv::noArray(), cv::noArray());

    std::cout << "views=" << imagePoints.size()
              << " size=" << imageSize.width << 'x' << imageSize.height
              << " rms_reproj_error=" << rms << " px\n"
              << "camera_matrix=\n" << cameraMatrix << '\n'
              << "dist_coeffs=" << distCoeffs << '\n';

    const std::filesystem::path output(kOutputPath);
    std::filesystem::create_directories(output.parent_path());
    cv::FileStorage storage(kOutputPath, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open output file for writing: " + std::string(kOutputPath));
    }
    storage << "camera_matrix" << cameraMatrix;
    storage << "dist_coeffs" << distCoeffs;
    storage.release();
    std::cout << "saved=" << kOutputPath << '\n';
    return 0;
}

int runImages(const std::string& imagesDir)
{
    namespace fs = std::filesystem;
    if (!fs::exists(imagesDir) || !fs::is_directory(imagesDir))
    {
        throw std::runtime_error("images directory not found: " + imagesDir);
    }

    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(imagesDir))
    {
        if (entry.is_regular_file())
        {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());

    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    for (const auto& file : files)
    {
        const cv::Mat image = cv::imread(file, cv::IMREAD_COLOR);
        if (image.empty())
        {
            continue;
        }
        const cv::Mat gray = toGray(image);
        std::vector<cv::Point2f> corners;
        if (detectCorners(gray, corners, true))
        {
            imagePoints.push_back(corners);
            imageSize = gray.size();
            std::cout << "ok   " << file << '\n';
        }
        else
        {
            std::cout << "skip " << file << " (no chessboard)\n";
        }
    }

    return calibrateAndSave(imagePoints, imageSize);
}

int runLive()
{
    // 构造即初始化，失败抛异常（由 main 的 try/catch 兜住）
    auto_aim::HikCamera camera;

    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;

    std::cout << "live calibration: SPACE accept view, u undo last, ENTER calibrate, ESC quit\n";

    while (true)
    {
        auto_aim::HikCameraFrame frame;
        const int grabResult = camera.capture(frame, kTimeoutMs);
        if (grabResult != MV_OK)
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        cv::Mat preview;
        cv::resize(frame.image, preview, {}, kPreviewScale, kPreviewScale, cv::INTER_AREA);
        const cv::Mat previewGray = toGray(preview);
        std::vector<cv::Point2f> corners;
        const bool found = detectCorners(previewGray, corners, false);

        cv::Mat display;
        if (preview.channels() == 1)
        {
            cv::cvtColor(preview, display, cv::COLOR_GRAY2BGR);
        }
        else
        {
            display = preview;
        }
        cv::drawChessboardCorners(display, kPatternSize, corners, found);
        cv::putText(
            display,
            "views=" + std::to_string(imagePoints.size()) + "/" + std::to_string(kRequiredViews) +
                (found ? "  board:OK" : "  board:--"),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            break;
        }
        if (key == 13 || key == 10)
        {
            if (imagePoints.size() < kRequiredViews)
            {
                std::cout << "need " << kRequiredViews << " views, got " << imagePoints.size() << '\n';
                continue;
            }
            return calibrateAndSave(imagePoints, imageSize);
        }
        if (key == ' ')
        {
            const cv::Mat gray = toGray(frame.image);
            std::vector<cv::Point2f> refined;
            if (detectCorners(gray, refined, true))
            {
                imagePoints.push_back(refined);
                imageSize = gray.size();
                std::cout << "accepted view " << imagePoints.size() << '\n';
            }
            else
            {
                std::cout << "no chessboard detected, view not accepted\n";
            }
        }
        else if ((key == 'u' || key == 'U') && !imagePoints.empty())
        {
            imagePoints.pop_back();
            std::cout << "removed last view, now " << imagePoints.size() << '\n';
        }
    }

    cv::destroyAllWindows();
    std::cout << "aborted, no file written\n";
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    try
    {
        if (argc == 1)
        {
            return runLive();
        }
        if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"))
        {
            printUsage(argv[0]);
            return 0;
        }
        if (argc == 3 && std::string(argv[1]) == "--images")
        {
            return runImages(argv[2]);
        }
        throw std::runtime_error("invalid arguments");
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
}
