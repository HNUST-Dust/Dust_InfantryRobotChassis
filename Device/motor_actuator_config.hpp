#pragma once

#include <cstdint>

#include "actuator/actuator_config.hpp"

namespace motor_cfg {

struct Config {
    static constexpr uint8_t kMaxActuators = 8;

    // index-based actuator list
    actuator::Item items[kMaxActuators] = {
        // 0..3 wheels (DJI)
        actuator::Item{.type = actuator::Type::DjiC620, .bus = actuator::Bus::CAN1, .std_id = 0x201, .dji_group_std_id = 0x200, .default_mode = actuator::Mode::Omega,
                       .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::MotorBus::CAN1, .rx_std_id = 0x201, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
        actuator::Item{.type = actuator::Type::DjiC620, .bus = actuator::Bus::CAN1, .std_id = 0x202, .dji_group_std_id = 0x200, .default_mode = actuator::Mode::Omega,
                       .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::MotorBus::CAN1, .rx_std_id = 0x202, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
        actuator::Item{.type = actuator::Type::DjiC620, .bus = actuator::Bus::CAN1, .std_id = 0x203, .dji_group_std_id = 0x200, .default_mode = actuator::Mode::Omega,
                       .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::MotorBus::CAN1, .rx_std_id = 0x203, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},
        actuator::Item{.type = actuator::Type::DjiC620, .bus = actuator::Bus::CAN1, .std_id = 0x204, .dji_group_std_id = 0x200, .default_mode = actuator::Mode::Omega,
                       .dji = actuator::drivers::DjiC6xxMin::Config{.bus = orb::MotorBus::CAN1, .rx_std_id = 0x204, .method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega}},

        // 4 yaw (DM)
        actuator::Item{.type = actuator::Type::DmNormal, .bus = actuator::Bus::CAN3, .std_id = 0x12, .dji_group_std_id = 0x1FF, .default_mode = actuator::Mode::Angle,
                       .dm = actuator::drivers::DmMitMin::Config{.bus = orb::DmBus::CAN3, .can_rx_id = 0x12, .master_id = 0x01, .angle_max = 12.56637f}},

        // 5 pitch (DM)
        actuator::Item{.type = actuator::Type::DmNormal, .bus = actuator::Bus::CAN3, .std_id = 0x11, .dji_group_std_id = 0x1FF, .default_mode = actuator::Mode::Angle,
                       .dm = actuator::drivers::DmMitMin::Config{.bus = orb::DmBus::CAN3, .can_rx_id = 0x11, .master_id = 0x02}},
    };

    // bring-up delays (ms)
    uint32_t delay_save_zero_ms = 1000;
    uint32_t delay_clear_error_ms = 1000;
    uint32_t delay_enter_ms = 1000;
};

} // namespace motor_cfg
