#include "usbcamera/usbcamera.h"
#include "logger.hpp"
#include <opencv2/opencv.hpp>

// 相机异常检测与恢复
void checkCameraStatus(USBCamera& camera) {
    // 检测相机是否打开
    if (!camera.isOpened()) {
        LOG_ERROR("相机连接断开，尝试重新连接...");
        camera.close();
        camera.open(); // 重新打开相机
        if (camera.isOpened()) {
            LOG_INFO("相机重新连接成功");
        } else {
            LOG_FATAL("相机重新连接失败，程序退出");
            exit(1);
        }
    }

    // 检测相机帧率是否正常（低于10fps视为异常）
    double current_fps = camera.getFPS();
    if (current_fps < 10.0) {
        LOG_WARN("相机帧率过低（{:.1f}fps），调整分辨率至640x480", current_fps);
        camera.setResolution(640, 480); // 降低分辨率提升帧率
        LOG_INFO("相机分辨率已调整，当前帧率：{:.1f}fps", camera.getFPS());
    }

    // 检测图像是否为空（避免采集失败导致程序崩溃）
    cv::Mat frame;
    camera.read(frame);
    if (frame.empty()) {
        LOG_ERROR("相机采集图像为空，尝试重置相机");
        camera.close();
        camera.open();
    }
}
