#pragma once

#include <cstdint>

#include "topic.hpp"

namespace orb {

// 业务层电机话题（app-level）：
// - 不暴露 CAN bus / StdId
// - MotorActuatorTask 在初始化时根据 motor_cfg::Config 完成映射

// ===== 底盘（DJI 轮电机） =====

struct ChassisWheelOmegaCmd {
    // 0..3 对应 4 个轮
    uint8_t wheel = 0;
    float omega = 0.0f;
};

inline RingTopic<ChassisWheelOmegaCmd, 32> chassis_wheel_omega_cmd{};

// ===== 云台（DM） =====

struct GimbalDmTarget {
    float yaw_angle = 0.0f;
    float yaw_omega = 0.0f;
    float yaw_torque = 0.0f;

    float pitch_angle = 0.0f;
    float pitch_omega = 0.0f;
    float pitch_torque = 0.0f;

    // MIT 参数（可选，不填默认 0）
    float kp = 0.0f;
    float kd = 0.0f;
};

inline RingTopic<GimbalDmTarget, 16> gimbal_dm_target{};

enum class GimbalDmAdminOp : uint8_t {
    None = 0,

    YawEnter,
    YawExit,
    YawClearError,
    YawSaveZero,

    PitchEnter,
    PitchExit,
    PitchClearError,

    BothEnter,
    BothExit,
    BothClearError,
};

struct GimbalDmAdminCmd {
    GimbalDmAdminOp op = GimbalDmAdminOp::None;
};

inline RingTopic<GimbalDmAdminCmd, 8> gimbal_dm_admin_cmd{};

} // namespace orb
