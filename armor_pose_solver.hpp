#ifndef ARMOR_POSE_SOLVER_HPP
#define ARMOR_POSE_SOLVER_HPP

#include <string>
#include <opencv2/opencv.hpp>

class ArmorPoseSolver {
public:
    ArmorPoseSolver(const std::string& config_path);
    void solve(const cv::Rect& armor_rect, float& distance, float& yaw, float& pitch);
};

#endif // ARMOR_POSE_SOLVER_HPP
