#pragma once

#include "topic.hpp"

// 通用 CAN Tx Topic：所有模块只负责“描述要发的帧”，由统一 CanTxTask 串行发送。
// 约束：该 Topic 是驱动层统一出口，上层模块不应直接调用 bsp_can_send。

namespace orb {

// CAN 总线抽象：CAN1/2/3
enum class CanBus : uint8_t {
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

enum class CanIdType : uint8_t {
    Std = 0,
    Ext = 1,
};

enum class CanFrameType : uint8_t {
    Data = 0,
    Remote = 1,
};

struct CanTxFrame {
    CanBus bus = CanBus::CAN1;

    uint32_t id = 0;      // 11-bit (STD) or 29-bit (EXT)
    uint8_t len = 0;     // 0..64
    uint8_t data[64] = {0};

    CanIdType id_type = CanIdType::Std;
    CanFrameType frame_type = CanFrameType::Data;

    bool is_fd = false;
    bool brs = false;
};

// 发送队列：发布即唤醒 CanTxTask
inline RingTopic<CanTxFrame, 32> can_tx;

}  // namespace orb
