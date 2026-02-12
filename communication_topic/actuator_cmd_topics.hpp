#pragma once

#include "topic.hpp"
#include "can_topics.hpp"

// 执行器/电机控制指令 Topic（与上层业务语义解耦）
// - 上层（Chassis/Gimbal/…）只需要发布“某电机的目标/管理指令”
// - 电机驱动仅按 bus + motor id 过滤，不感知 chassis/gimbal/yaw/pitch 等概念

namespace orb {

// ===================== DJI C6xx =====================
// 按电机反馈 STD ID 定位电机（如 0x201..0x208）。
struct DjiC6xxOmegaCmd {
    CanBus bus = CanBus::CAN1;
    uint16_t rx_std_id = 0x201;
    float omega = 0.0f; // 输出轴角速度（rad/s）
};

inline RingTopic<DjiC6xxOmegaCmd, 32> dji_c6xx_omega_cmd;

// ===================== DM MIT =====================
// can_rx_id 仅取低 4-bit（0..15），对应电机反馈 ID 的低位。
struct DmMitTargetCmd {
    CanBus bus = CanBus::CAN1;
    uint8_t can_rx_id = 0x01;

    float angle = 0.0f;
    float omega = 0.0f;
    float torque = 0.0f;

    float kp = 0.0f;
    float kd = 0.0f;
};

inline RingTopic<DmMitTargetCmd, 16> dm_mit_target_cmd;

enum class DmMitAdminOp : uint8_t {
    Enter = 0,
    Exit,
    ClearError,
    SaveZero,
};

struct DmMitAdminCmd {
    CanBus bus = CanBus::CAN1;
    uint8_t can_rx_id = 0x01;
    DmMitAdminOp op = DmMitAdminOp::Enter;
};

inline RingTopic<DmMitAdminCmd, 8> dm_mit_admin_cmd;

} // namespace orb
