#ifndef GIMBAL_CONTROLLER_HPP
#define GIMBAL_CONTROLLER_HPP

class GimbalController {
public:
    GimbalController();
    void calculateControl(float target_yaw, float target_pitch, 
                         float current_yaw, float current_pitch,
                         float& control_yaw, float& control_pitch);
    void reset();
};

#endif // GIMBAL_CONTROLLER_HPP
