#include <algorithm>
#include <atomic>
#include <csignal>
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

#include "hik_capture.hpp"

namespace
{

std::atomic_bool g_stop{false};

constexpr unsigned int kCameraIndex = 0;
constexpr int kTimeoutMs = 1000;
constexpr int kRequiredViews = 20;
constexpr double kSquareSize = 25.0;
constexpr char kOutputPath[] = "config/camera_calibration.yml";
const cv::Size kPatternSize(9, 6);

void handleSignal(int)
{
    g_stop = true;
}

void printUsage(const char* exe)
{
    std::cout
        << "Usage:\n"
        << "  " << exe << "\n"
        << "  " << exe << " --images DIR\n\n"
        << "Calibrate a camera with a 9x6-inner-corner chessboard using at least 20 views.\n"
        << "The square size is 25 mm and the result is written to " << kOutputPath << ".\n\n"
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
    const std::vector<std::vector<cv::Point2f>>& image_points,
    cv::Size image_size)
{
    if (image_points.size() < kRequiredViews)
    {
        std::cerr << "need at least " << kRequiredViews
                  << " detected views to calibrate, got " << image_points.size() << '\n';
        return 1;
    }

    const std::vector<cv::Point3f> object = makeObjectPoints();
    const std::vector<std::vector<cv::Point3f>> object_points(image_points.size(), object);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    const double rms = cv::calibrateCamera(
        object_points, image_points, image_size, camera_matrix, dist_coeffs,
        cv::noArray(), cv::noArray());

    std::cout << "views=" << image_points.size()
              << " size=" << image_size.width << 'x' << image_size.height
              << " rms_reproj_error=" << rms << " px\n"
              << "camera_matrix=\n" << camera_matrix << '\n'
              << "dist_coeffs=" << dist_coeffs << '\n';

    const std::filesystem::path output(kOutputPath);
    std::filesystem::create_directories(output.parent_path());
    cv::FileStorage storage(kOutputPath, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open output file for writing: " + std::string(kOutputPath));
    }
    storage << "camera_matrix" << camera_matrix;
    storage << "dist_coeffs" << dist_coeffs;
    storage.release();
    std::cout << "saved=" << kOutputPath << '\n';
    return 0;
}

int runImages(const std::string& images_dir)
{
    namespace fs = std::filesystem;
    if (!fs::exists(images_dir) || !fs::is_directory(images_dir))
    {
        throw std::runtime_error("images directory not found: " + images_dir);
    }

    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(images_dir))
    {
        if (entry.is_regular_file())
        {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());

    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;
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
            image_points.push_back(corners);
            image_size = gray.size();
            std::cout << "ok   " << file << '\n';
        }
        else
        {
            std::cout << "skip " << file << " (no chessboard)\n";
        }
    }

    return calibrateAndSave(image_points, image_size);
}

int runLive()
{
    auto_aim::HikCapture camera;
    const auto& devices = camera.devices();
    if (devices.empty())
    {
        std::cerr << "no Hikrobot camera found\n";
        return 2;
    }

    camera.open(kCameraIndex, {});

    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;

    std::cout << "live calibration: SPACE accept view, u undo last, ENTER calibrate, ESC quit\n";

    while (!g_stop)
    {
        auto_aim::HikFrame frame;
        if (!camera.grab(frame, kTimeoutMs))
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        const cv::Mat gray = toGray(frame.image);
        std::vector<cv::Point2f> corners;
        const bool found = detectCorners(gray, corners, false);

        cv::Mat preview;
        if (frame.image.channels() == 1)
        {
            cv::cvtColor(frame.image, preview, cv::COLOR_GRAY2BGR);
        }
        else
        {
            preview = frame.image.clone();
        }
        cv::drawChessboardCorners(preview, kPatternSize, corners, found);
        cv::putText(
            preview,
            "views=" + std::to_string(image_points.size()) + "/" + std::to_string(kRequiredViews) +
                (found ? "  board:OK" : "  board:--"),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow("calibrate_camera", preview);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            break;
        }
        if (key == 13 || key == 10)
        {
            if (image_points.size() < kRequiredViews)
            {
                std::cout << "need " << kRequiredViews << " views, got " << image_points.size() << '\n';
                continue;
            }
            return calibrateAndSave(image_points, image_size);
        }
        if (key == ' ')
        {
            std::vector<cv::Point2f> refined;
            if (detectCorners(gray, refined, true))
            {
                image_points.push_back(refined);
                image_size = gray.size();
                std::cout << "accepted view " << image_points.size() << '\n';
            }
            else
            {
                std::cout << "no chessboard detected, view not accepted\n";
            }
        }
        else if ((key == 'u' || key == 'U') && !image_points.empty())
        {
            image_points.pop_back();
            std::cout << "removed last view, now " << image_points.size() << '\n';
        }
    }

    std::cout << "aborted, no file written\n";
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

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
