#include "yaml.hpp"
#include "logger.hpp"
#include "serial/serial.h"
#include <iostream>
#include <fmt/core.h>

class SerialComm {
private:
    serial::Serial ser;
    std::string port;      // 串口端口（如"/dev/ttyUSB0"，树莓派需根据实际修改）
    int baudrate;          // 波特率（如115200，与下位机一致）
    bool is_open;          // 串口打开状态

public:
    // 初始化通信，复用yaml读取配置参数
    SerialComm(const std::string& config_path) {
        is_open = false;
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            port = config["serial"]["port"].as<std::string>();
            baudrate = config["serial"]["baudrate"].as<int>();

            // 配置串口参数
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 超时时间1000ms
            ser.setTimeout(to);
            ser.open();

            if (ser.isOpen()) {
                is_open = true;
                LOG_INFO("云台串口通信初始化成功，端口：{}，波特率：{}", port, baudrate);
            } else {
                LOG_ERROR("串口已配置，但无法打开端口：{}", port);
            }
        } catch (YAML::Exception& e) {
            LOG_ERROR("读取串口配置文件失败：{}", e.what());
        } catch (serial::IOException& e) {
            LOG_ERROR("串口初始化失败：{}", e.what());
        }
    }

    // 发送装甲板位姿数据至下位机（yaw：偏航角，pitch：俯仰角，distance：距离）
    void sendArmorPose(float yaw, float pitch, float distance) {
        if (!is_open || !ser.isOpen()) {
            LOG_ERROR("串口未打开，无法发送数据");
            return;
        }
        // 数据打包格式（可根据下位机协议修改，此处为逗号分隔，换行结束）
        std::string data = fmt::format("{:.2f},{:.2f},{:.2f}\n", yaw, pitch, distance);
        ser.write(data);
        LOG_DEBUG("发送云台控制数据：{}", data);
    }

    // 接收下位机云台状态反馈（如当前云台角度、运行状态）
    std::string receiveGimbalStatus() {
        if (!is_open || !ser.isOpen()) {
            LOG_ERROR("串口未打开，无法接收数据");
            return "";
        }
        if (ser.available() > 0) {
            std::string feedback = ser.readline(); // 读取一行反馈数据
            LOG_DEBUG("接收云台状态：{}", feedback);
            return feedback;
        }
        return "";
    }

    // 检查串口状态
    bool isSerialOpen() const {
        return is_open && ser.isOpen();
    }

    // 关闭串口
    ~SerialComm() {
        if (ser.isOpen()) {
            ser.close();
            LOG_INFO("串口已关闭");
        }
    }
};
