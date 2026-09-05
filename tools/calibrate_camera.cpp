#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>
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
constexpr double kSquareSize = 5.0;
constexpr double kMaxViewError = 1.0;  // 单张重投影误差(px)超过此值视为离群，剔除后重标
constexpr double kMaxHandEyeRotError = 1.0;     // 单位姿棋盘→世界旋转偏差(度)超此值视为离群
constexpr double kMaxHandEyeTransError = 10.0;  // 单位姿棋盘→世界平移偏差(mm)超此值视为离群
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

// 全分辨率找棋盘：只在采集结束后的批处理里对拍下的图逐张调用，不进预览循环，故慢无妨
bool detectCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners)
{
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    return cv::findChessboardCorners(gray, kPatternSize, corners, flags);
}

// 采纳时对已检出的角点原地做亚像素细化，供 calibrateCamera / solvePnP 用
void refineCorners(const cv::Mat& gray, std::vector<cv::Point2f>& corners)
{
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
}

// 标定并保存内参。imagePoints 会被就地裁成「重投影误差达标」的子集：
// 达标张数够(>=kRequiredViews)就用干净子集重标+保存并返回 true；
// 不够则返回 false，让调用方继续采图补齐。
bool calibrateAndSave(
    std::vector<std::vector<cv::Point2f>>& imagePoints,
    cv::Size imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs)
{
    const std::vector<cv::Point3f> object = makeObjectPoints();
    const std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), object);

    // 首次标定：额外要每张视图的重投影误差，用来定位离群张
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat perViewErrors;
    double rms = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,
        rvecs, tvecs, cv::noArray(), cv::noArray(), perViewErrors);

    // 打印每张视图误差，一眼看出是个别糊帧拖垮还是整体都差
    std::cout << "rms_reproj_error=" << rms << " px, per-view:\n";
    for (int i = 0; i < perViewErrors.rows; ++i)
    {
        std::cout << "  view " << i << " = " << perViewErrors.at<double>(i) << " px\n";
    }

    // 只保留误差达标的视图，原地更新 imagePoints；被剔的打印出来
    std::vector<std::vector<cv::Point2f>> kept;
    for (int i = 0; i < perViewErrors.rows; ++i)
    {
        if (perViewErrors.at<double>(i) <= kMaxViewError)
        {
            kept.push_back(imagePoints[static_cast<std::size_t>(i)]);
        }
        else
        {
            std::cout << "drop view " << i << " (" << perViewErrors.at<double>(i)
                      << " px > " << kMaxViewError << " px)\n";
        }
    }
    imagePoints = kept;

    // 达标的不够，别标定别保存，回去继续采图补齐
    if (imagePoints.size() < kRequiredViews)
    {
        std::cout << "clean views " << imagePoints.size() << "/" << kRequiredViews
                  << ", keep capturing\n";
        return false;
    }

    // 用干净子集重标一次，这才是要保存的内参
    const std::vector<std::vector<cv::Point3f>> keptObject(imagePoints.size(), object);
    rms = cv::calibrateCamera(
        keptObject, imagePoints, imageSize, cameraMatrix, distCoeffs,
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
    return true;
}

// 解手眼并保存。四个位姿向量会被就地裁成「棋盘→世界自洽」的子集：
// 达标位姿够(>=kRequiredHandEyeViews)就用干净子集重解+保存并返回 true；
// 不够则返回 false，让调用方继续采集补齐。
bool solveAndSaveHandEye(
    std::vector<cv::Mat>& rGripper2Base,
    std::vector<cv::Mat>& tGripper2Base,
    std::vector<cv::Mat>& rTarget2Cam,
    std::vector<cv::Mat>& tTarget2Cam)
{
    cv::Mat rCam2Gripper;
    cv::Mat tCam2Gripper;

    // 解 → 评估自洽性 → 剔离群 → 剩下够就重解，不够就停。循环里评估代码只写一份
    while (true)
    {
        // 解手眼：eye-in-hand，得相机→云台机体的旋转 X=cam→gripper + 平移
        cv::calibrateHandEye(
            rGripper2Base, tGripper2Base, rTarget2Cam, tTarget2Cam,
            rCam2Gripper, tCam2Gripper, cv::CALIB_HAND_EYE_TSAI);

        // 质量评估(方案二)：板子固定，每帧反推的棋盘→世界位姿理应重合，看离散度
        // T_target2base = T_gripper2base · X · T_target2cam
        const std::size_t n = rTarget2Cam.size();
        std::vector<cv::Mat> rTb(n);
        std::vector<cv::Vec3d> tTb(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            cv::Mat rTemp = rCam2Gripper * rTarget2Cam[i];                    // R_c2g · R_t2c
            cv::Mat tTemp = rCam2Gripper * tTarget2Cam[i] + tCam2Gripper;     // R_c2g · t_t2c + t_c2g
            rTb[i] = rGripper2Base[i] * rTemp;                               // R_g2b · 上
            cv::Mat tb = rGripper2Base[i] * tTemp + tGripper2Base[i];        // R_g2b · 上 + t_g2b(=0)
            tTb[i] = cv::Vec3d(tb.at<double>(0), tb.at<double>(1), tb.at<double>(2));
        }

        // 平移均值
        cv::Vec3d tMean(0, 0, 0);
        for (std::size_t i = 0; i < n; ++i)
        {
            tMean += tTb[i];
        }
        tMean *= 1.0 / static_cast<double>(n);

        // 旋转均值：旋转矩阵求和后 SVD 正交化
        Eigen::Matrix3d rSum = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < n; ++i)
        {
            Eigen::Matrix3d e;
            cv::cv2eigen(rTb[i], e);
            rSum += e;
        }
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(rSum, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d rMeanE = svd.matrixU() * svd.matrixV().transpose();
        cv::Mat rMean;
        cv::eigen2cv(rMeanE, rMean);

        // 逐帧偏差：平移欧氏距离(mm)、旋转与均值的夹角(度)
        std::vector<double> rotDev(n);
        std::vector<double> transDev(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            transDev[i] = cv::norm(tTb[i] - tMean);
            cv::Mat dr = rMean.t() * rTb[i];
            const double trace = dr.at<double>(0, 0) + dr.at<double>(1, 1) + dr.at<double>(2, 2);
            rotDev[i] = std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / CV_PI;
        }

        // 整体离散度(std)当质量数，再逐帧打印
        double rotStd = 0.0;
        double transStd = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            rotStd += rotDev[i] * rotDev[i];
            transStd += transDev[i] * transDev[i];
        }
        rotStd = std::sqrt(rotStd / static_cast<double>(n));
        transStd = std::sqrt(transStd / static_cast<double>(n));
        std::cout << "handeye poses=" << n << " target2base spread: rot_std=" << rotStd
                  << " deg, trans_std=" << transStd << " mm, per-pose:\n";
        for (std::size_t i = 0; i < n; ++i)
        {
            std::cout << "  pose " << i << " rot=" << rotDev[i] << " deg trans=" << transDev[i] << " mm\n";
        }

        // 只保留自洽达标的位姿
        std::vector<cv::Mat> kRg;
        std::vector<cv::Mat> kTg;
        std::vector<cv::Mat> kRt;
        std::vector<cv::Mat> kTt;
        for (std::size_t i = 0; i < n; ++i)
        {
            if (rotDev[i] <= kMaxHandEyeRotError && transDev[i] <= kMaxHandEyeTransError)
            {
                kRg.push_back(rGripper2Base[i]);
                kTg.push_back(tGripper2Base[i]);
                kRt.push_back(rTarget2Cam[i]);
                kTt.push_back(tTarget2Cam[i]);
            }
            else
            {
                std::cout << "drop pose " << i << " (rot=" << rotDev[i] << " deg, trans="
                          << transDev[i] << " mm)\n";
            }
        }

        // 没有离群：当前解就是最终解，跳出去保存
        if (kRt.size() == n)
        {
            break;
        }

        // 有离群：更新为干净子集
        rGripper2Base = kRg;
        tGripper2Base = kTg;
        rTarget2Cam = kRt;
        tTarget2Cam = kTt;

        // 达标位姿不够，别保存，回去继续采
        if (rTarget2Cam.size() < kRequiredHandEyeViews)
        {
            std::cout << "clean poses " << rTarget2Cam.size() << "/" << kRequiredHandEyeViews
                      << ", keep capturing\n";
            return false;
        }
        // 否则用干净子集重解一轮
    }

    std::cout << "cam_to_gimbal_rotation=\n" << rCam2Gripper << '\n'
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
    return true;
}

int run()
{
    // 构造即初始化，失败抛异常：相机 + 串口(IMU 四元数)
    auto_aim::hik_camera::HikCamera camera;
    auto_aim::serial::Serial serial;

    // 预览用全分辨率显示，窗口可缩放且保持宽高比；给个 1080p 屏上留边的初始尺寸
    cv::namedWindow("calibrate_camera", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("calibrate_camera", 1280, 800);

    // === 阶段一：内参标定 ===
    // 采集时不检测角点，预览只管流畅显示；SPACE 拍一张存起来，ENTER 后再对所有拍下的图批量找角点
    std::vector<std::vector<cv::Point2f>> imagePoints;  // 已成功检出的视图，跨批累积到够数
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> shots;  // 本批拍下的全分辨率灰度图，ENTER 批量检测后清空

    std::cout << "hand-eye stage 1/2 (intrinsics): SPACE capture, u undo, ENTER detect+calibrate, ESC quit\n";

    bool intrinsicsDone = false;
    while (!intrinsicsDone)
    {
        auto_aim::hik_camera::HikCameraFrame frame;
        const int grabResult = camera.capture(frame);
        if (grabResult != MV_OK)
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        // 预览：直接显示当前帧，只叠张数进度，不做任何检测，故不卡
        cv::Mat display = frame.image.clone();
        cv::putText(
            display,
            "captured=" + std::to_string(shots.size()) +
                "  accepted=" + std::to_string(imagePoints.size()) + "/" + std::to_string(kRequiredViews),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            cv::destroyAllWindows();
            std::cout << "aborted, no file written\n";
            return 0;
        }
        if (key == ' ')
        {
            // 拍一张：只存全分辨率灰度图，检测推迟到 ENTER
            shots.push_back(toGray(frame.image));
            imageSize = shots.back().size();
            std::cout << "captured " << shots.size() << '\n';
        }
        else if ((key == 'u' || key == 'U') && !shots.empty())
        {
            shots.pop_back();
            std::cout << "removed last shot, now " << shots.size() << '\n';
        }
        else if (key == 13 || key == 10)
        {
            // 批量找角点：本批逐张检测，成功的做亚像素细化后并入 imagePoints，再清空本批
            int found = 0;
            for (std::size_t i = 0; i < shots.size(); ++i)
            {
                std::vector<cv::Point2f> corners;
                if (detectCorners(shots[i], corners))
                {
                    refineCorners(shots[i], corners);
                    imagePoints.push_back(corners);
                    ++found;
                }
                std::cout << "detect " << (i + 1) << "/" << shots.size() << " ok=" << found << '\r' << std::flush;
            }
            std::cout << "\ndetected " << found << "/" << shots.size()
                      << ", total accepted " << imagePoints.size() << '\n';
            shots.clear();

            if (imagePoints.size() < kRequiredViews)
            {
                std::cout << "need " << kRequiredViews << " views, keep capturing\n";
                continue;
            }
            // 达标视图够就标定+保存并结束；不够 calibrateAndSave 会裁掉坏张并提示，留在循环继续采
            if (calibrateAndSave(imagePoints, imageSize, cameraMatrix, distCoeffs))
            {
                intrinsicsDone = true;
            }
        }
    }

    // === 阶段二：手眼采集 ===
    // 棋盘物点：手眼阶段用它跑 solvePnP 得棋盘→相机外参
    const std::vector<cv::Point3f> object = makeObjectPoints();
    std::vector<cv::Mat> rGripper2Base;
    std::vector<cv::Mat> tGripper2Base;
    std::vector<cv::Mat> rTarget2Cam;
    std::vector<cv::Mat> tTarget2Cam;

    // 采集时同样只拍不检测；每张图要连同当下的云台四元数一起存，供 ENTER 批处理解算
    std::vector<cv::Mat> heShots;
    std::vector<auto_aim::serial::Quaternion> heQuats;

    std::cout << "hand-eye stage 2/2: 每次采集前转动云台改变朝向; "
                 "SPACE capture, u undo, ENTER detect+solve, ESC quit\n";

    while (true)
    {
        auto_aim::hik_camera::HikCameraFrame frame;
        const int grabResult = camera.capture(frame);
        if (grabResult != MV_OK)
        {
            std::cerr << "warning: frame timeout/error\n";
            continue;
        }

        // 预览：显示当前帧 + 实时四元数 + 张数，不做检测
        const auto_aim::serial::Quaternion liveQ = serial.latest();
        cv::Mat display = frame.image.clone();
        cv::putText(
            display,
            "captured=" + std::to_string(heShots.size()) +
                "  accepted=" + std::to_string(rTarget2Cam.size()) + "/" + std::to_string(kRequiredHandEyeViews),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        char quatText[96];
        std::snprintf(quatText, sizeof(quatText), "quat w=%.3f x=%.3f y=%.3f z=%.3f", liveQ.w, liveQ.x, liveQ.y, liveQ.z);
        cv::putText(
            display, quatText, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6,
            cv::Scalar(255, 255, 0), 2);
        cv::imshow("calibrate_camera", display);

        const int key = cv::waitKey(1);
        if (key == 27)
        {
            break;
        }
        if (key == ' ')
        {
            // 拍一张：灰度图 + 当下四元数配对存起来，检测与解算推迟到 ENTER
            heShots.push_back(toGray(frame.image));
            heQuats.push_back(liveQ);
            std::cout << "captured " << heShots.size() << '\n';
        }
        else if ((key == 'u' || key == 'U') && !heShots.empty())
        {
            heShots.pop_back();
            heQuats.pop_back();
            std::cout << "removed last shot, now " << heShots.size() << '\n';
        }
        else if (key == 13 || key == 10)
        {
            // 批量处理：本批逐张找角点，成功的 solvePnP 得棋盘→相机、四元数转机体→世界，并入解算集合
            int found = 0;
            for (std::size_t i = 0; i < heShots.size(); ++i)
            {
                std::vector<cv::Point2f> corners;
                if (detectCorners(heShots[i], corners))
                {
                    refineCorners(heShots[i], corners);

                    // 棋盘→相机：solvePnP 得 rvec/tvec，Rodrigues 转旋转矩阵
                    cv::Mat rvec;
                    cv::Mat tvec;
                    cv::solvePnP(object, corners, cameraMatrix, distCoeffs, rvec, tvec);
                    cv::Mat rTarget;
                    cv::Rodrigues(rvec, rTarget);

                    // 机体→世界(=gripper→base)：IMU 四元数经 Eigen 转旋转矩阵，平移置 0（云台绕固定轴旋转）
                    Eigen::Quaterniond qImu(heQuats[i].w, heQuats[i].x, heQuats[i].y, heQuats[i].z);
                    Eigen::Matrix3d rImu = qImu.normalized().toRotationMatrix();
                    cv::Mat rGripper;
                    cv::eigen2cv(rImu, rGripper);

                    rTarget2Cam.push_back(rTarget);
                    tTarget2Cam.push_back(tvec);
                    rGripper2Base.push_back(rGripper);
                    tGripper2Base.push_back(cv::Mat::zeros(3, 1, CV_64F));
                    ++found;
                }
                std::cout << "detect " << (i + 1) << "/" << heShots.size() << " ok=" << found << '\r' << std::flush;
            }
            std::cout << "\ndetected " << found << "/" << heShots.size()
                      << ", total accepted " << rTarget2Cam.size() << '\n';
            heShots.clear();
            heQuats.clear();

            if (rTarget2Cam.size() < kRequiredHandEyeViews)
            {
                std::cout << "need " << kRequiredHandEyeViews << " poses, keep capturing\n";
                continue;
            }
            // 达标位姿够就解算+保存并结束；不够 solveAndSaveHandEye 会裁掉离群位姿并提示，留在循环继续采
            if (solveAndSaveHandEye(rGripper2Base, tGripper2Base, rTarget2Cam, tTarget2Cam))
            {
                return 0;
            }
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
