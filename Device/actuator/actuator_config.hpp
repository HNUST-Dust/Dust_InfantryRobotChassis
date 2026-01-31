#pragma once

#include <cstdint>

#include "actuator_iface.hpp"
#include "drivers/dji_c6xx_min.hpp"
#include "drivers/dm_mit_min.hpp"

namespace actuator {

enum class Type : uint8_t {
    None = 0,
    DjiC620,
    DmNormal,
};

enum class Bus : uint8_t {
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

struct Item {
    Type type = Type::None;
    Bus bus = Bus::CAN1;

    // for CAN dispatch lookup (std id)
    uint32_t std_id = 0;

    // DJI group current frame id (0x200/0x1FF/0x2FF). Used when type==DjiC620.
    uint16_t dji_group_std_id = 0x200;

    // default command mode (optional)
    Mode default_mode = Mode::Disabled;

    // type-specific config (minimal drivers)
    drivers::DjiC6xxMin::Config dji{};
    drivers::DmMitMin::Config dm{};
};

} // namespace actuator
