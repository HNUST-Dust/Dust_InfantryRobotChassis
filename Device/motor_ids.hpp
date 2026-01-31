#pragma once

#include <cstdint>

namespace motor_ids {

// DJI wheel motors on CAN1 (0x201~0x204)
static constexpr uint32_t kWheel1 = 0x201;
static constexpr uint32_t kWheel2 = 0x202;
static constexpr uint32_t kWheel3 = 0x203;
static constexpr uint32_t kWheel4 = 0x204;

// DM gimbal motors on CAN3
static constexpr uint32_t kGimbalYaw = 0x12;
static constexpr uint32_t kGimbalPitch = 0x11;

// Supercap on CAN3
static constexpr uint32_t kSupercap = 0x100;

} // namespace motor_ids
