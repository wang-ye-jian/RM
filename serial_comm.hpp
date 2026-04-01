#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <string>

class SerialComm {
private:
    class Impl; // 隐藏实现细节
    Impl* pImpl;

public:
    SerialComm(const std::string& config_path);
    ~SerialComm();
    void sendArmorPose(float yaw, float pitch, float distance);
    std::string receiveGimbalStatus();
    bool isSerialOpen() const;
};

#endif // SERIAL_COMM_HPP
