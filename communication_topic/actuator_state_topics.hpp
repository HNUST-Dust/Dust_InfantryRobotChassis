#pragma once

#include <cstdint>

#include "can_topics.hpp"
#include "topic.hpp"

namespace orb {

// DM MIT 电机反馈（来自电机 8B 反馈帧解包）
// - angle_rad：解包角度（环绕值，范围约为 [-angle_max, angle_max]）
// - total_angle_rad：在驱动内通过环绕展开得到的累计角
// - omega_rad_s / torque_nm：解包值
// - temp：原始字节（具体单位依电机协议）
struct DmMitFeedback {
    CanBus bus = CanBus::CAN1;
    uint8_t can_rx_id = 0x01;

    float angle_rad = 0.0f;
    float total_angle_rad = 0.0f;
    float omega_rad_s = 0.0f;
    float torque_nm = 0.0f;

    uint8_t mos_temp = 0;
    uint8_t rotor_temp = 0;
};

// MultiTopic 索引：bus(1..3) + can_rx_id(0..15)
// 约定：idx = (bus-1)*16 + (can_rx_id & 0x0F)，范围 [0, 47]
constexpr int dm_mit_feedback_index(CanBus bus, uint8_t can_rx_id)
{
    const int bus_i = static_cast<int>(bus);
    if (bus_i < 1 || bus_i > 3) {
        return (can_rx_id & 0x0F);
    }
    return (bus_i - 1) * 16 + (can_rx_id & 0x0F);
}

inline MultiTopic<DmMitFeedback, 48> dm_mit_feedback;

} // namespace orb
