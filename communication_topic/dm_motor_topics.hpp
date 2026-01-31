#pragma once

#include "topic.hpp"

// DM 电机 Tx 语义化 Topic：替代在驱动内部直接 can_send_data()
// 先覆盖 dvc_motor_dm.cpp 当前用到的几类帧：
// - Normal 模式：MIT / AngleOmega / Omega / EMIT（不同 payload 长度 4/8）
// - 通用指令：Enter / Exit / ClearError / SaveZero（固定 8B payload）
// - 1-to-4 模式：0x3FE / 0x4FE 四路电流组帧（每路 int16，8B）

namespace orb {

enum class MotorBus : uint8_t;  // 复用 motor_topics.hpp 中的枚举

enum class DmBus : uint8_t {
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

// DM 电机发送帧（语义化：DM 控制输出）。
// 注意：这里仍然以“最终可直接发的 payload”为语义边界，避免上层看到 HAL / FDCAN。
struct DmTxFrame {
    DmBus bus = DmBus::CAN1;
    uint16_t std_id = 0;
    uint8_t len = 8;          // 4 or 8
    uint8_t data[8] = {0};
};

// 发送队列：发布即唤醒 CAN TxTask
inline RingTopic<DmTxFrame, 16> dm_tx_frame;

// DM 1-to-4 电流组帧：四路 int16（8B）。通常 std_id 为 0x3FE 或 0x4FE。
struct Dm1To4CurrentGroupCmd {
    DmBus bus = DmBus::CAN1;
    uint16_t std_id = 0x3FE;  // 0x3FE / 0x4FE
    int16_t current[4] = {0, 0, 0, 0};
};

inline RingTopic<Dm1To4CurrentGroupCmd, 8> dm_1to4_current_group_cmd;

}  // namespace orb
