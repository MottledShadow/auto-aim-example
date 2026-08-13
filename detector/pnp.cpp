#include "pnp.hpp"

#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

namespace auto_aim
{

CameraCalibration loadCalibration(const std::string& path)
{
    CameraCalibration calibration;

    //空路径直接返回空标定（error 为空，交给调用方判断 camera_matrix 是否可用）
    if (path.empty())
    {
        return calibration;
    }

    try
    {
        //打开 OpenCV YAML，读不到就抛异常走下面的 catch
        cv::FileStorage storage(path, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw std::runtime_error("cannot open calibration file: " + path);
        }

        //读内参矩阵与畸变系数，缺任一项都算失败
        storage["camera_matrix"] >> calibration.camera_matrix;
        storage["dist_coeffs"] >> calibration.dist_coeffs;
        if (calibration.camera_matrix.empty() || calibration.dist_coeffs.empty())
        {
            throw std::runtime_error("calibration file must contain camera_matrix and dist_coeffs");
        }

        //内参统一成 3x3 的 CV_64F，形状不对就报错
        calibration.camera_matrix.convertTo(calibration.camera_matrix, CV_64F);
        if (calibration.camera_matrix.rows != 3 || calibration.camera_matrix.cols != 3)
        {
            throw std::runtime_error("camera_matrix must be 3x3");
        }

        //畸变系数统一成单行的 CV_64F
        calibration.dist_coeffs.convertTo(calibration.dist_coeffs, CV_64F);
        calibration.dist_coeffs = calibration.dist_coeffs.reshape(1, 1).clone();
    }
    catch (const std::exception& ex)
    {
        //加载失败：释放半成品矩阵，把原因记进 error
        calibration.camera_matrix.release();
        calibration.dist_coeffs.release();
        calibration.error = ex.what();
    }

    return calibration;
}

PnpSolver::PnpSolver(const CameraCalibration& calibration, const PnpSolverParams& params)
    : calibration_(calibration), params_(params)
{
}

std::vector<ArmorPose> PnpSolver::solve(const std::vector<Armor>& armors) const
{
    std::vector<ArmorPose> poses;

    //没有可用内参（标定失败或未加载）就返回空结果
    if (calibration_.camera_matrix.empty())
    {
        return poses;
    }

    for (const Armor& armor : armors)
    {
        //2D 图像点：左上→右上→右下→左下（与灯条上下端点契约一致）
        const std::vector<cv::Point2f> image_points = {
            armor.left_light.top,
            armor.right_light.top,
            armor.right_light.bottom,
            armor.left_light.bottom,
        };

        //3D 物体点：按大小装甲取物理尺寸(mm)，同样按左上→右上→右下→左下排列
        const bool is_large = armor.type == ArmorType::Large;
        const float half_width = static_cast<float>((is_large ? params_.large_armor_width : params_.small_armor_width) * 0.5);
        const float half_height = static_cast<float>((is_large ? params_.large_armor_height : params_.small_armor_height) * 0.5);
        const std::vector<cv::Point3f> object_points = {
            {-half_width, -half_height, 0.0F},
            { half_width, -half_height, 0.0F},
            { half_width,  half_height, 0.0F},
            {-half_width,  half_height, 0.0F},
        };

        //解算位姿，失败的装甲板跳过
        cv::Mat rvec;
        cv::Mat tvec;
        const bool solved = cv::solvePnP(
            object_points,
            image_points,
            calibration_.camera_matrix,
            calibration_.dist_coeffs,
            rvec,
            tvec,
            false,
            params_.solve_pnp_method);
        if (!solved)
        {
            continue;
        }

        //组装该装甲板的位姿结果
        ArmorPose pose;
        pose.armor = armor;
        pose.rvec = rvec;
        pose.tvec = tvec;
        poses.push_back(pose);
    }

    return poses;
}

} // namespace auto_aim
