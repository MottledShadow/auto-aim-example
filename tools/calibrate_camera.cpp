#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "hik_camera.hpp"
#include "serial.hpp"

namespace
{

constexpr int kRequiredViews = 20;
constexpr std::size_t kRequiredHandEyeViews = 15;
constexpr double kSquareSize = 60.0;
constexpr int kDetectInterval = 3;   // 每 3 帧才在原图上检测一次角点，其余帧沿用上次结果
constexpr char kOutputPath[] = "config/camera_calibration.yml";
constexpr char kHandEyeOutputPath[] = "config/hand_eye_calibration.yml";
const cv::Size kPatternSize(11, 8);

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

// 预览判断和采纳共用这一份结果，避免两次检测分辨率不同导致的不一致
bool detectCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners)
{
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    return cv::findChessboardCorners(gray, kPatternSize, corners, flags);
}

// 采纳时对已检出的角点原地做亚像素细化，供 calibrateCamera / solvePnP 用
void refineCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners)
{
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
}

int calibrateAndSave(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    cv::Size imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    if (imagePoints.size() < kRequiredViews)
    {
        std::cerr << "need at least " << kRequiredViews
                  << " detected views to calibrate, got " << imagePoints.size() << '\n';
        return 1;
    }

    const std::vector<cv::Point3f> object = makeObjectPoints();
    const std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), object);

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

int solveAndSaveHandEye(
    const std::vector<cv::Mat>& rGripper2Base,
    const std::vector<cv::Mat>& tGripper2Base,
    const std::vector<cv::Mat>& rTarget2Cam,
    const std::vector<cv::Mat>& tTarget2Cam)
{
    // 解手眼：eye-in-hand，得相机→云台机体的旋转+平移
    cv::Mat rCam2Gripper;
    cv::Mat tCam2Gripper;
    cv::calibrateHandEye(
        rGripper2Base, tGripper2Base, rTarget2Cam, tTarget2Cam,
        rCam2Gripper, tCam2Gripper, cv::CALIB_HAND_EYE_TSAI);

    std::cout << "handeye poses=" << rTarget2Cam.size() << '\n'
              << "cam_to_gimbal_rotation=\n" << rCam2Gripper << '\n'
              << "cam_to_gimbal_translation=" << tCam2Gripper.t() << " mm\n";

    const std::filesystem::path output(kHandEyeOutputPath);
    std::filesystem::create_directories(output.parent_path());
    cv::FileStorage storage(kHandEyeOutputPath, cv::FileStorage::WRITE);
    if (!storage.isOpened())
    {
        throw std::runtime_error("cannot open output file for writing: " + std::string(kHandEyeOutputPath));
    }
    storage << "cam_to_gimbal_rotation" << rCam2Gripper;
    storage << "cam_to_gimbal_translation" << tCam2Gripper;
    storage << "method" << "TSAI";
    storage << "views" << static_cast<int>(rTarget2Cam.size());
    storage.release();
    std::cout << "saved=" << kHandEyeOutputPath << '\n';
    return 0;
}

int run()
{
    // 构造即初始化，失败抛异常：相机 + 串口(IMU 四元数)
    auto_aim::HikCamera camera;
    Serial serial;

    // 预览用全分辨率显示，窗口设成可缩放以免超出屏幕
    cv::namedWindow("calibrate_camera", cv::WINDOW_NORMAL);

    // === 阶段一：内参标定===
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    std::cout << "hand-eye stage 1/2 (intrinsics): SPACE accept view, u undo, ENTER calibrate, ESC quit\n";

    // 隔帧检测：每 kDetectInterval 帧才在原图跑一次角点检测，其余帧沿用缓存
    // 缓存整帧的灰度图、角点、是否检出，以及画好角点的预览底图，保证看到的==采纳的
    int sinceDetect = kDetectInterval;
    cv::Mat detGray;
    std::vector<cv::Point2f> detCorners;
    bool detFound = false;
    cv::Mat detBase;

    bool intrinsicsDone = false;
    while (!intrinsicsDone)
    {
        auto_aim::HikCameraFrame frame;
        const int grabResult = camera.capture(frame);
        if (grabResult != MV_OK)
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        // 到间隔就在全分辨率原图上检测一次，刷新缓存与预览底图
        if (++sinceDetect >= kDetectInterval)
        {
            sinceDetect = 0;
            detGray = toGray(frame.image);
            detFound = detectCorners(detGray, detCorners);
            detBase = frame.image.clone();
            cv::drawChessboardCorners(detBase, kPatternSize, detCorners, detFound);
        }

        // 预览底图上叠当前进度文字（文字每帧刷新，底图按检测间隔刷新）
        cv::Mat display = detBase.clone();
        cv::putText(
            display,
            "intrinsics views=" + std::to_string(imagePoints.size()) + "/" + std::to_string(kRequiredViews) +
                (detFound ? "  board:OK" : "  board:--"),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            detFound ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            cv::destroyAllWindows();
            std::cout << "aborted, no file written\n";
            return 0;
        }
        if (key == 13 || key == 10)
        {
            if (imagePoints.size() < kRequiredViews)
            {
                std::cout << "need " << kRequiredViews << " views, got " << imagePoints.size() << '\n';
                continue;
            }
            const int rc = calibrateAndSave(imagePoints, imageSize, cameraMatrix, distCoeffs);
            if (rc != 0)
            {
                return rc;
            }
            intrinsicsDone = true;
        }
        else if (key == ' ')
        {
            // 采纳的就是预览里那一帧检测结果，仅补做亚像素细化
            if (!detFound)
            {
                std::cout << "no chessboard detected, view not accepted\n";
                continue;
            }
            refineCorners(detGray, detCorners);
            imagePoints.push_back(detCorners);
            imageSize = detGray.size();
            std::cout << "accepted view " << imagePoints.size() << '\n';
        }
        else if ((key == 'u' || key == 'U') && !imagePoints.empty())
        {
            imagePoints.pop_back();
            std::cout << "removed last view, now " << imagePoints.size() << '\n';
        }
    }

    // === 阶段二：手眼采集 ===
    // 棋盘物点：手眼阶段用它跑 solvePnP 得棋盘→相机外参
    const std::vector<cv::Point3f> object = makeObjectPoints();
    std::vector<cv::Mat> rGripper2Base;
    std::vector<cv::Mat> tGripper2Base;
    std::vector<cv::Mat> rTarget2Cam;
    std::vector<cv::Mat> tTarget2Cam;

    std::cout << "hand-eye stage 2/2: 每次采集前转动云台改变朝向; "
                 "SPACE accept pose, u undo, ENTER solve, ESC quit\n";

    // 同阶段一的隔帧检测；额外缓存检测那一刻的云台四元数，保证解算用的姿态与画面同步
    sinceDetect = kDetectInterval;
    Quaternion detQ{};

    while (true)
    {
        auto_aim::HikCameraFrame frame;
        const int grabResult = camera.capture(frame);
        if (grabResult != MV_OK)
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        // 到间隔就在全分辨率原图上检测一次，同步抓当前四元数，刷新缓存与预览底图
        if (++sinceDetect >= kDetectInterval)
        {
            sinceDetect = 0;
            detGray = toGray(frame.image);
            detFound = detectCorners(detGray, detCorners);
            detQ = serial.latest();

            if (frame.image.channels() == 1)
            {
                cv::cvtColor(frame.image, detBase, cv::COLOR_GRAY2BGR);
            }
            else
            {
                detBase = frame.image.clone();
            }
            cv::drawChessboardCorners(detBase, kPatternSize, detCorners, detFound);
        }

        cv::Mat display = detBase.clone();
        cv::putText(
            display,
            "handeye poses=" + std::to_string(rTarget2Cam.size()) + "/" + std::to_string(kRequiredHandEyeViews) +
                (detFound ? "  board:OK" : "  board:--"),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
            detFound ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        char quatText[96];
        std::snprintf(quatText, sizeof(quatText), "quat w=%.3f x=%.3f y=%.3f z=%.3f", detQ.w, detQ.x, detQ.y, detQ.z);
        cv::putText(
            display, quatText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(255, 255, 0), 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            break;
        }
        if (key == 13 || key == 10)
        {
            if (rTarget2Cam.size() < kRequiredHandEyeViews)
            {
                std::cout << "need " << kRequiredHandEyeViews << " poses, got " << rTarget2Cam.size() << '\n';
                continue;
            }
            return solveAndSaveHandEye(rGripper2Base, tGripper2Base, rTarget2Cam, tTarget2Cam);
        }
        if (key == ' ')
        {
            // 采纳的就是预览里那一帧的角点与四元数，仅补做亚像素细化
            if (!detFound)
            {
                std::cout << "no chessboard detected, pose not accepted\n";
                continue;
            }
            refineCorners(detGray, detCorners);

            // 棋盘→相机：solvePnP 得 rvec/tvec，Rodrigues 转旋转矩阵
            cv::Mat rvec;
            cv::Mat tvec;
            cv::solvePnP(object, detCorners, cameraMatrix, distCoeffs, rvec, tvec);
            cv::Mat rTarget;
            cv::Rodrigues(rvec, rTarget);

            // 机体→世界(=gripper→base)：IMU 四元数经 Eigen 转旋转矩阵，平移置 0（云台绕固定轴旋转）
            Eigen::Quaterniond qImu(detQ.w, detQ.x, detQ.y, detQ.z);
            Eigen::Matrix3d rImu = qImu.normalized().toRotationMatrix();
            cv::Mat rGripper;
            cv::eigen2cv(rImu, rGripper);

            rTarget2Cam.push_back(rTarget);
            tTarget2Cam.push_back(tvec);
            rGripper2Base.push_back(rGripper);
            tGripper2Base.push_back(cv::Mat::zeros(3, 1, CV_64F));
            std::cout << "accepted pose " << rTarget2Cam.size() << '\n';
        }
        else if ((key == 'u' || key == 'U') && !rTarget2Cam.empty())
        {
            rTarget2Cam.pop_back();
            tTarget2Cam.pop_back();
            rGripper2Base.pop_back();
            tGripper2Base.pop_back();
            std::cout << "removed last pose, now " << rTarget2Cam.size() << '\n';
        }
    }

    cv::destroyAllWindows();
    std::cout << "aborted, no hand-eye file written\n";
    return 0;
}

} // namespace

int main()
try
{
    return run();
}
catch (const std::exception& ex)
{
    std::cerr << "error: " << ex.what() << '\n';
    return 1;
}
