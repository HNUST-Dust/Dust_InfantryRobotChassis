#pragma once

#include "topic.hpp"

// 该文件用于集中定义与外设/外部模块（如 supercap/referee）相关的 Topic 数据结构与实例。

namespace orb {

// ===================== supercap =====================
// 超级电容充放电模式（强类型，避免依赖 legacy supercap.h）
enum class SupercapChargeMode : uint8_t {
    Discharge = 0,
    Charge = 1,
};

// 超级电容接收数据（来自超电板 -> 主控）
struct SupercapRx {
    uint8_t supercap_work_status = 0;
    uint8_t supercap_status_code = 0;
    uint8_t supercap_energy_percent = 0;
    uint8_t chassis_compensate_power = 0;
    uint8_t battery_voltage = 0;
};

// 超级电容控制数据（主控 -> 超电板）
struct SupercapTx {
    uint8_t supercap_enable_status = 0;
    SupercapChargeMode supercap_charge_status = SupercapChargeMode::Charge;
    uint8_t power_limit_max = 0;
    uint8_t charge_power = 0;
};

// 使用 RingTopic 作为发送队列，支持“发布即唤醒发送任务”
inline Topic<SupercapRx> supercap_rx;
inline RingTopic<SupercapTx, 4> supercap_tx;

// ===================== referee =====================
// 裁判系统里最常用且体量较小的一部分状态（可按需继续扩展）
struct RefereeStatus {
    uint8_t id = 0;
    uint8_t level = 0;
    uint16_t current_hp = 0;
    uint16_t max_hp = 0;
    uint16_t shooter_barrel_cooling_value = 0;
    uint16_t shooter_barrel_heat_limit = 0;
    uint16_t chassis_power_limit = 0;
    uint8_t power_management_gimbal_output = 0;
    uint8_t power_management_chassis_output = 0;
    uint8_t power_management_shooter_output = 0;
};

struct RefereeShoot {
    uint8_t bullet_type = 0;
    uint8_t shooter_number = 0;
    uint8_t launching_frequency = 0;
    float initial_speed = 0.0f;
};

inline Topic<RefereeStatus> referee_status;
inline Topic<RefereeShoot> referee_shoot;

} // namespace orb
