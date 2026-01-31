#pragma once

#include "topic.hpp"

// Robot <-> Gimbal 解耦 Topic
// - Robot 作为发布者：发布 gimbal_cmd
// - Gimbal 作为订阅者：copy 最新 cmd 并驱动自身控制
// 
// 说明：
// 1) 这里用 Topic<T>（只保留最新一帧），符合“控制指令只关心最新值”的语义。
// 2) 事件驱动由 register_notifier 的 Notifier 完成（Robot publish -> 唤醒 Gimbal Task）。

namespace orb {

enum class GimbalYawMode : uint8_t {
    Angle = 0,  // 角度环（位置）
    Omega = 1,  // 速度环
};

struct GimbalCmd {
    // 传感器输入（来自 Robot 聚合）
    float yaw_imu_angle = 0.0f;
    float yaw_imu_omega = 0.0f;
    float pitch_imu_angle = 0.0f;
    float pitch_imu_omega = 0.0f;

    // 操作手/自瞄融合后的“虚拟目标角”
    float virtual_yaw_angle = 0.0f;
    float virtual_pitch_angle = 0.0f;

    // 控制模式 & 前馈
    GimbalYawMode yaw_mode = GimbalYawMode::Angle;
    float yaw_omega_ff = 0.0f;

    // yaw 速度目标（当 yaw_mode==Omega 时使用）
    float target_yaw_omega = 0.0f;

    // 命令：是否请求置零（保持阻塞特性由上层决定；gimbal 内部按命令触发动作）
    bool request_yaw_zero = false;

    // 命令：yaw 电机恢复（ClearError + Enter）
    bool request_yaw_recover = false;

    // 命令：pitch 电机恢复（ClearError + Enter）
    bool request_pitch_recover = false;

    // 命令：云台双轴恢复（BothRecover）
    bool request_gimbal_recover = false;

    // 命令：退出（急停/疯车保护时可用）
    bool request_exit = false;
};

inline Topic<GimbalCmd> gimbal_cmd;

}  // namespace orb
