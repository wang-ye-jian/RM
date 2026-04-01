#include "yaml.hpp"
#include "logger.hpp"
#include <Eigen/Dense>
#include <string>

// 配置参数结构体（统一管理所有配置）
struct ProjectConfig {
    // 相机参数
    int camera_width;    // 相机分辨率宽度
    int camera_height;   // 相机分辨率高度
    int camera_fps;      // 相机帧率
    Eigen::Matrix3d camera_intrinsic;  // 相机内参
    Eigen::Vector4d camera_distortion;// 相机畸变参数

    // 装甲板参数
    float armor_width;   // 装甲板实际宽度（m）
    float armor_height;  // 装甲板实际高度（m）

    // 串口通信参数
    std::string serial_port; // 串口端口
    int serial_baudrate;     // 串口波特率

    // 识别参数
    int armor_threshold; // 装甲板识别阈值
};

// 读取配置文件，返回配置参数
ProjectConfig loadConfig(const std::string& config_path) {
    ProjectConfig config;
    try {
        YAML::Node root = YAML::LoadFile(config_path);

        // 读取相机参数
        config.camera_width = root["camera"]["width"].as<int>();
        config.camera_height = root["camera"]["height"].as<int>();
        config.camera_fps = root["camera"]["fps"].as<int>();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                config.camera_intrinsic(i, j) = root["camera"]["intrinsic"][i][j].as<double>();
            }
        }
        for (int i = 0; i < 4; ++i) {
            config.camera_distortion(i) = root["camera"]["distortion"][i].as<double>();
        }

        // 读取装甲板参数
        config.armor_width = root["armor"]["width"].as<float>();
        config.armor_height = root["armor"]["height"].as<float>();

        // 读取串口参数
        config.serial_port = root["serial"]["port"].as<std::string>();
        config.serial_baudrate = root["serial"]["baudrate"].as<int>();

        // 读取识别参数
        config.armor_threshold = root["recognition"]["threshold"].as<int>();

        LOG_INFO("配置文件读取成功，路径：{}", config_path);
    } catch (YAML::Exception& e) {
        LOG_ERROR("配置文件读取失败：{}", e.what());
        exit(1);
    }
    return config;
}
