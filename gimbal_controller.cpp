#include "logger.hpp"
#include "math_tools.hpp"
#include <cmath>

class GimbalController {
private:
    // PID参数（可通过yaml配置，此处先定义默认值，后续可优化）
    float kp_yaw;    // 偏航角比例系数
    float ki_yaw;    // 偏航角积分系数
    float kd_yaw;    // 偏航角微分系数
    float kp_pitch;  // 俯仰角比例系数
    float ki_pitch;  // 俯仰角积分系数
    float kd_pitch;  // 俯仰角微分系数

    // PID控制中间变量
    float error_yaw;      // 当前偏航角偏差（目标角 - 当前角）
    float error_pitch;    // 当前俯仰角偏差
    float integral_yaw;   // 偏航角积分值
    float integral_pitch; // 俯仰角积分值
    float last_error_yaw; // 上一次偏航角偏差
    float last_error_pitch;// 上一次俯仰角偏差

    // 积分饱和限制（防止积分溢出）
    const float integral_max = 100.0f;
    const float integral_min = -100.0f;

    // 云台控制量限制（适配云台硬件，可修改）
    const float control_max = 10.0f;
    const float control_min = -10.0f;

public:
    // 初始化PID参数（可后续改为从yaml读取）
    GimbalController() 
        : kp_yaw(0.5f), ki_yaw(0.01f), kd_yaw(0.1f),
          kp_pitch(0.5f), ki_pitch(0.01f), kd_pitch(0.1f),
          error_yaw(0.0f), error_pitch(0.0f),
          integral_yaw(0.0f), integral_pitch(0.0f),
          last_error_yaw(0.0f), last_error_pitch(0.0f) {
        LOG_INFO("云台PID控制器初始化成功，默认PID参数已加载");
    }

    // 计算云台控制量（输入：目标yaw/pitch、当前云台yaw/pitch；输出：云台转动控制量）
    void calculateControl(float target_yaw, float target_pitch, 
                         float current_yaw, float current_pitch,
                         float& control_yaw, float& control_pitch) {
        // 1. 计算当前偏差
        error_yaw = target_yaw - current_yaw;
        error_pitch = target_pitch - current_pitch;

        // 2. 计算积分（限制积分饱和）
        integral_yaw += error_yaw;
        integral_pitch += error_pitch;
        integral_yaw = math_tools::clamp(integral_yaw, integral_min, integral_max);
        integral_pitch = math_tools::clamp(integral_pitch, integral_min, integral_max);

        // 3. 计算微分（当前偏差 - 上一次偏差）
        float diff_yaw = error_yaw - last_error_yaw;
        float diff_pitch = error_pitch - last_error_pitch;

        // 4. 计算PID控制量
        control_yaw = kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * diff_yaw;
        control_pitch = kp_pitch * error_pitch + ki_pitch * integral_pitch + kd_pitch * diff_pitch;

        // 5. 限制控制量范围（适配云台硬件）
        control_yaw = math_tools::clamp(control_yaw, control_min, control_max);
        control_pitch = math_tools::clamp(control_pitch, control_min, control_max);

        // 6. 更新上一次偏差
        last_error_yaw = error_yaw;
        last_error_pitch = error_pitch;

        LOG_DEBUG("云台控制量计算：yaw={:.2f}，pitch={:.2f}", control_yaw, control_pitch);
    }

    // 重置PID控制器（云台复位时使用）
    void reset() {
        error_yaw = 0.0f;
        error_pitch = 0.0f;
        integral_yaw = 0.0f;
        integral_pitch = 0.0f;
        last_error_yaw = 0.0f;
        last_error_pitch = 0.0f;
        LOG_DEBUG("云台PID控制器已重置");
    }
};
