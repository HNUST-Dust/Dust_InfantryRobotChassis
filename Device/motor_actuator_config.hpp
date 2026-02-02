#pragma once

#include <cstdint>

#include "../communication_topic/can_topics.hpp"

#include "actuator/drivers/dji_c6xx_min.hpp"
#include "actuator/drivers/dm_mit_min.hpp"

namespace motor_cfg {

enum class Type : uint8_t {
    None = 0,
    DjiC620,
    DmNormal,
};

struct Item {
    Type type = Type::None;
    orb::CanBus bus = orb::CanBus::CAN1;

    // CAN std id (DJI: 0x201..; DM: 0x11/0x12)
    uint16_t std_id = 0;

    // optional: DJI group current frame id (0x200/0x1FF/0x2FF)
    uint16_t dji_group_std_id = 0x200;

    // type-specific config
    actuator::drivers::DjiC6xxMin::Config dji{};
    actuator::drivers::DmMitMin::Config dm{};
};

struct Config {
    static constexpr uint8_t kMaxActuators = 8;

    // ===== 业务映射（业务层不需要知道 CAN bus/std_id） =====
    // chassis wheels: 0..3 -> items[] index
    uint8_t chassis_wheel_index[4] = {0, 1, 2, 3};
    // gimbal DM motors -> items[] index
    uint8_t gimbal_yaw_index = 4;
    uint8_t gimbal_pitch_index = 5;

    // index-based actuator list
    Item items[kMaxActuators] = {
        // 0..3 wheels (DJI)
           Item{.type = Type::DjiC620, .bus = orb::CanBus::CAN1, .std_id = 0x201, .dji_group_std_id = 0x200,
               .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::CanBus::CAN1, .rx_std_id = 0x201, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
           Item{.type = Type::DjiC620, .bus = orb::CanBus::CAN1, .std_id = 0x202, .dji_group_std_id = 0x200,
               .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::CanBus::CAN1, .rx_std_id = 0x202, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
           Item{.type = Type::DjiC620, .bus = orb::CanBus::CAN1, .std_id = 0x203, .dji_group_std_id = 0x200,
               .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::CanBus::CAN1, .rx_std_id = 0x203, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
           Item{.type = Type::DjiC620, .bus = orb::CanBus::CAN1, .std_id = 0x204, .dji_group_std_id = 0x200,
               .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::CanBus::CAN1, .rx_std_id = 0x204, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},

        // 4 yaw (DM)
           Item{.type = Type::DmNormal, .bus = orb::CanBus::CAN3, .std_id = 0x12, .dji_group_std_id = 0x1FF,
               .dm = actuator::drivers::DmMitMin::Config{.bus = orb::CanBus::CAN3, .can_rx_id = 0x12, .master_id = 0x01, .angle_max = 12.56637f}},

        // 5 pitch (DM)
           Item{.type = Type::DmNormal, .bus = orb::CanBus::CAN3, .std_id = 0x11, .dji_group_std_id = 0x1FF,
               .dm = actuator::drivers::DmMitMin::Config{.bus = orb::CanBus::CAN3, .can_rx_id = 0x11, .master_id = 0x02}},
    };

    // bring-up delays (ms)
    uint32_t delay_save_zero_ms = 1000;
    uint32_t delay_clear_error_ms = 1000;
    uint32_t delay_enter_ms = 1000;
};

} // namespace motor_cfg
