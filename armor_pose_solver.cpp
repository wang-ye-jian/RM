#include "math_tools.hpp"
#include "yaml.hpp"
#include "logger.hpp"
#include "img_tools.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class ArmorPoseSolver {
private:
    Eigen::Matrix3d camera_intrinsic;  // 相机内参矩阵（3x3）
    Eigen::Vector4d camera_distortion; // 相机畸变参数（k1, k2, p1, p2）
    float armor_width;                 // 装甲板实际宽度（单位：m，需根据实际规格修改）
    float armor_height;                // 装甲板实际高度（单位：m）

public:
    // 初始化，复用yaml读取相机参数和装甲板规格
    ArmorPoseSolver(const std::string& config_path) {
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            // 读取装甲板实际尺寸（从配置文件获取，无需硬编码）
            armor_width = config["armor"]["width"].as<float>();
            armor_height = config["armor"]["height"].as<float>();

            // 读取相机内参（3x3矩阵）
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    camera_intrinsic(i, j) = config["camera"]["intrinsic"][i][j].as<double>();
                }
            }

            // 读取相机畸变参数（k1, k2, p1, p2）
            camera_distortion << config["camera"]["distortion"][0].as<double>(),
                                config["camera"]["distortion"][1].as<double>(),
                                config["camera"]["distortion"][2].as<double>(),
                                config["camera"]["distortion"][3].as<double>();

            LOG_INFO("装甲板位姿解算模块初始化成功");
        } catch (YAML::Exception& e) {
            LOG_ERROR("读取位姿解算配置失败：{}", e.what());
            exit(1);
        }
    }

    // 解算装甲板位姿（输入：装甲板在图像中的矩形区域，输出：距离、偏航角、俯仰角）
    void solve(const cv::Rect& armor_rect, float& distance, float& yaw, float& pitch) {
        // 1. 获取装甲板中心像素坐标
        cv::Point2f armor_center(
            armor_rect.x + armor_rect.width / 2.0f,
            armor_rect.y + armor_rect.height / 2.0f
        );

        // 2. 像素坐标转相机坐标（复用math_tools中的坐标转换函数）
        Eigen::Vector3d camera_point = math_tools::pixel2camera(
            armor_center, camera_intrinsic, camera_distortion
        );

        // 3. 计算装甲板到相机的实际距离（欧氏距离）
        distance = sqrt(
            camera_point.x() * camera_point.x() +
            camera_point.y() * camera_point.y() +
            camera_point.z() * camera_point.z()
        );

        // 4. 计算偏航角yaw（弧度转角度，复用math_tools的rad2deg函数）
        yaw = math_tools::rad2deg(atan2(camera_point.y(), camera_point.x()));

        // 5. 计算俯仰角pitch（弧度转角度）
        float horizontal_dist = sqrt(camera_point.x() * camera_point.x() + camera_point.y() * camera_point.y());
        pitch = math_tools::rad2deg(atan2(camera_point.z(), horizontal_dist));

        LOG_DEBUG("位姿解算结果：距离={:.2f}m，yaw={:.2f}°，pitch={:.2f}°", distance, yaw, pitch);
    }
};
