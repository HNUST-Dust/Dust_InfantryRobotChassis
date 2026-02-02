#pragma once

#include "topic.hpp"

// Gimbal -> 其他模块状态 Topic
// 用于切断对 Gimbal 对象的直接读取依赖（通过订阅状态 Topic 获取姿态）。
//
// 说明：这里用 Topic<T> 只保留最新值，适合状态类数据。

namespace orb {

struct GimbalState {
    // yaw/pitch 当前状态（单位：rad / rad/s）
    float yaw_angle = 0.0f;
    float yaw_omega = 0.0f;
    float pitch_angle = 0.0f;
    float pitch_omega = 0.0f;

    // 非累计角（按现有代码语义）
    float yaw_angle_noncumulative = 0.0f;
    float pitch_angle_noncumulative = 0.0f;
};

inline Topic<GimbalState> gimbal_state;

}  // namespace orb
