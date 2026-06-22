#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
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

void handleSignal(int)
{
    g_stop = true;
}

struct CalibOptions
{
    unsigned int index = 0;
    int cols = 9;
    int rows = 6;
    double square_size = 25.0;
    unsigned int min_views = 15;
    int timeout_ms = 1000;
    std::string output = "config/camera_calibration.yml";
    std::string images_dir;
    bool list_only = false;
};

bool startsWith(const std::string& text, const std::string& prefix)
{
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

std::string takeValue(int& i, int argc, char** argv, const std::string& arg, const std::string& key)
{
    const std::string with_equals = key + "=";
    if (startsWith(arg, with_equals))
    {
        return arg.substr(with_equals.size());
    }
    if (arg == key && i + 1 < argc)
    {
        return argv[++i];
    }
    throw std::runtime_error("missing value for " + key);
}

unsigned int parseUInt(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    unsigned long parsed = std::stoul(value, &pos, 10);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid integer for " + key + ": " + value);
    }
    return static_cast<unsigned int>(parsed);
}

double parseDouble(const std::string& value, const std::string& key)
{
    std::size_t pos = 0;
    double parsed = std::stod(value, &pos);
    if (pos != value.size())
    {
        throw std::runtime_error("invalid number for " + key + ": " + value);
    }
    return parsed;
}

void printUsage(const char* exe)
{
    std::cout
        << "Usage: " << exe << " [options]\n\n"
        << "Chessboard camera calibration. Writes camera_matrix/dist_coeffs YAML\n"
        << "that the vision pipeline reads back via loadCalibration.\n\n"
        << "Options:\n"
        << "  --list                    list cameras and exit\n"
        << "  --index N                 camera index, default 0\n"
        << "  --cols N                  inner corners per row, default 9\n"
        << "  --rows N                  inner corners per column, default 6\n"
        << "  --square-size F           square size in mm, default 25 (intrinsics are scale-free)\n"
        << "  --min-views N             recommended views before calibrating, default 15\n"
        << "  --images DIR              calibrate from images in DIR instead of the live camera\n"
        << "  --timeout-ms N            frame timeout, default 1000\n"
        << "  --output PATH             output YAML, default config/camera_calibration.yml\n"
        << "  --help                    show this help\n\n"
        << "Live keys: SPACE/c accept view, u undo last, ENTER calibrate, ESC quit\n\n"
        << "Examples:\n"
        << "  " << exe << " --index 0 --cols 9 --rows 6 --square-size 25\n"
        << "  " << exe << " --images captures --cols 9 --rows 6\n";
}

CalibOptions parseArgs(int argc, char** argv)
{
    CalibOptions options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--list")
        {
            options.list_only = true;
        }
        else if (arg == "--index" || startsWith(arg, "--index="))
        {
            options.index = parseUInt(takeValue(i, argc, argv, arg, "--index"), "--index");
        }
        else if (arg == "--cols" || startsWith(arg, "--cols="))
        {
            options.cols = static_cast<int>(parseUInt(takeValue(i, argc, argv, arg, "--cols"), "--cols"));
        }
        else if (arg == "--rows" || startsWith(arg, "--rows="))
        {
            options.rows = static_cast<int>(parseUInt(takeValue(i, argc, argv, arg, "--rows"), "--rows"));
        }
        else if (arg == "--square-size" || startsWith(arg, "--square-size="))
        {
            options.square_size = parseDouble(takeValue(i, argc, argv, arg, "--square-size"), "--square-size");
        }
        else if (arg == "--min-views" || startsWith(arg, "--min-views="))
        {
            options.min_views = parseUInt(takeValue(i, argc, argv, arg, "--min-views"), "--min-views");
        }
        else if (arg == "--timeout-ms" || startsWith(arg, "--timeout-ms="))
        {
            options.timeout_ms = static_cast<int>(parseUInt(takeValue(i, argc, argv, arg, "--timeout-ms"), "--timeout-ms"));
        }
        else if (arg == "--output" || startsWith(arg, "--output="))
        {
            options.output = takeValue(i, argc, argv, arg, "--output");
        }
        else if (arg == "--images" || startsWith(arg, "--images="))
        {
            options.images_dir = takeValue(i, argc, argv, arg, "--images");
        }
        else
        {
            throw std::runtime_error("unknown option: " + arg);
        }
    }
    if (options.cols < 2 || options.rows < 2)
    {
        throw std::runtime_error("--cols and --rows must be at least 2 (inner corner counts)");
    }
    return options;
}

void printDevices(const std::vector<auto_aim::HikDeviceInfo>& devices)
{
    for (const auto& device : devices)
    {
        std::cout << '[' << device.index << "] " << device.transport
                  << "  model=" << device.model
                  << "  serial=" << device.serial
                  << "  accessible=" << (device.accessible ? "yes" : "no") << '\n';
    }
}

std::vector<cv::Point3f> makeObjectPoints(int cols, int rows, double square_size)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            points.emplace_back(static_cast<float>(c * square_size), static_cast<float>(r * square_size), 0.0F);
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

bool detectCorners(const cv::Mat& gray, cv::Size pattern, std::vector<cv::Point2f>& corners, bool refine)
{
    int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    if (!refine)
    {
        flags |= cv::CALIB_CB_FAST_CHECK;
    }
    if (!cv::findChessboardCorners(gray, pattern, corners, flags))
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
    cv::Size image_size,
    const CalibOptions& options)
{
    if (image_points.size() < 3)
    {
        std::cerr << "need at least 3 detected views to calibrate, got " << image_points.size() << '\n';
        return 1;
    }

    const std::vector<cv::Point3f> object = makeObjectPoints(options.cols, options.rows, options.square_size);
    const std::vector<std::vector<cv::Point3f>> object_points(image_points.size(), object);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    const double rms = cv::calibrateCamera(
        object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "views=" << image_points.size()
              << " size=" << image_size.width << 'x' << image_size.height
              << " rms_reproj_error=" << rms << " px\n";
    for (std::size_t i = 0; i < image_points.size(); ++i)
    {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected);
        const double err = cv::norm(image_points[i], projected, cv::NORM_L2) /
                           std::sqrt(static_cast<double>(projected.size()));
        std::cout << "  view " << i << " error=" << err << " px\n";
    }
    std::cout << "camera_matrix=\n" << camera_matrix << '\n';
    std::cout << "dist_coeffs=" << dist_coeffs << '\n';

    const std::filesystem::path out_path(options.output);
    if (out_path.has_parent_path())
    {
        std::filesystem::create_directories(out_path.parent_path());
    }
    cv::FileStorage storage(options.output, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open output file for writing: " + options.output);
    }
    storage << "camera_matrix" << camera_matrix;
    storage << "dist_coeffs" << dist_coeffs;
    storage.release();
    std::cout << "saved=" << options.output << '\n';
    return 0;
}

int runImages(const CalibOptions& options)
{
    namespace fs = std::filesystem;
    if (!fs::exists(options.images_dir) || !fs::is_directory(options.images_dir))
    {
        throw std::runtime_error("images directory not found: " + options.images_dir);
    }

    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(options.images_dir))
    {
        if (entry.is_regular_file())
        {
            files.push_back(entry.path().string());
        }
    }
    std::sort(files.begin(), files.end());

    const cv::Size pattern(options.cols, options.rows);
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
        if (detectCorners(gray, pattern, corners, true))
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

    if (image_points.size() < options.min_views)
    {
        std::cout << "warning: only " << image_points.size()
                  << " usable views (recommended >= " << options.min_views << ")\n";
    }
    return calibrateAndSave(image_points, image_size, options);
}

int runLive(const CalibOptions& options)
{
    auto_aim::HikCapture camera;
    const auto& devices = camera.devices();
    if (devices.empty())
    {
        std::cerr << "no Hikrobot camera found\n";
        return 2;
    }

    printDevices(devices);
    if (options.list_only)
    {
        return 0;
    }
    if (options.index >= devices.size())
    {
        std::cerr << "camera index out of range: " << options.index << '\n';
        return 2;
    }

    camera.open(options.index, {});

    const cv::Size pattern(options.cols, options.rows);
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;
    bool do_calibrate = false;

    std::cout << "live calibration: SPACE/c accept view, u undo last, ENTER calibrate, ESC quit\n";

    while (!g_stop)
    {
        auto_aim::HikFrame frame;
        if (!camera.grab(frame, options.timeout_ms))
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        const cv::Mat gray = toGray(frame.image);
        std::vector<cv::Point2f> corners;
        const bool found = detectCorners(gray, pattern, corners, false);

        cv::Mat preview;
        if (frame.image.channels() == 1)
        {
            cv::cvtColor(frame.image, preview, cv::COLOR_GRAY2BGR);
        }
        else
        {
            preview = frame.image.clone();
        }
        cv::drawChessboardCorners(preview, pattern, corners, found);
        cv::putText(
            preview,
            "views=" + std::to_string(image_points.size()) + (found ? "  board:OK" : "  board:--"),
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
            do_calibrate = true;
            break;
        }
        if (key == ' ' || key == 'c' || key == 'C')
        {
            std::vector<cv::Point2f> refined;
            if (detectCorners(gray, pattern, refined, true))
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

    if (!do_calibrate)
    {
        std::cout << "aborted, no file written\n";
        return 0;
    }
    if (image_points.size() < options.min_views)
    {
        std::cout << "warning: only " << image_points.size()
                  << " views (recommended >= " << options.min_views << ")\n";
    }
    return calibrateAndSave(image_points, image_size, options);
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try
    {
        const CalibOptions options = parseArgs(argc, argv);
        if (!options.images_dir.empty())
        {
            return runImages(options);
        }
        return runLive(options);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "error: " << ex.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
}
