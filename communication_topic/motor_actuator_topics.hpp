#pragma once

// 电机执行器层 Topic：把“业务目标(底盘/云台)”与“具体电机驱动实现”彻底解耦。
//
// - Chassis/Gimbal 只发布这些执行器命令，不再持有 MotorDji/MotorDm 对象。
// - MotorActuatorTask 订阅这些命令，更新 MotorDji/MotorDm，并 publish 到已有 CAN Tx Topics。

#include <cstdint>

#include "../communication_topic/topic_pubsub.hpp"

namespace orb {

// ===== DJI 3508/C620: wheel actuators =====
struct WheelSpeedCmd {
    // 0..3 => 4 wheels
    uint8_t wheel = 0;
    float omega = 0.0f;  // rad/s or driver expected unit (保持与原 SetTargetOmega 语义一致)
};

// Ring topic: wheel 的目标速度命令
inline RingTopic<WheelSpeedCmd, 16> wheel_speed_cmd{};

// ===== DM gimbal actuators =====
struct GimbalDmTarget {
    float yaw_angle = 0.0f;
    float yaw_omega = 0.0f;
    float yaw_torque = 0.0f;

    float pitch_angle = 0.0f;
    float pitch_omega = 0.0f;
    float pitch_torque = 0.0f;
};

inline RingTopic<GimbalDmTarget, 8> gimbal_dm_target{};

// ===== DM gimbal actuator admin commands =====
enum class GimbalDmAdminOp : uint8_t {
    None = 0,

    // 单轴动作
    YawSaveZero,
    YawRecover,      // ClearError + Enter
    PitchRecover,    // ClearError + Enter

    // 双轴动作
    BothExit,
    BothEnter,
    BothRecover,     // ClearError + Enter for both
};

struct GimbalDmAdminCmd {
    GimbalDmAdminOp op = GimbalDmAdminOp::None;
};

inline RingTopic<GimbalDmAdminCmd, 4> gimbal_dm_admin_cmd{};

}  // namespace orb
