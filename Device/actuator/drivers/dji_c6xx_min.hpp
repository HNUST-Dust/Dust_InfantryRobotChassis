#pragma once

#include <cstdint>

#include "../../../Platform/bsp_can_port.h"
#include "control/alg_pid.h"
#include "../../../communication_topic/can_topics.hpp"

namespace actuator::drivers {

class DjiC6xxMin final {
public:
    enum class ControlMethod : uint8_t {
        Current = 0,
        Omega,
    };

    struct Config {
        orb::CanBus bus = orb::CanBus::CAN1;
        uint16_t rx_std_id = 0x201;
        float gearbox_ratio = 1.0f;
        ControlMethod method = ControlMethod::Omega;

        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        float current_limit = 20.0f;
        uint16_t enc_per_round = 8192;
    };

    void Init(BspCanHandle can, const Config& cfg);

    // Registers this motor into a DJI group-current TX loop.
    // Slot mapping is derived from (rx_std_id - (group_std_id + 1)) in range [0..3].
    void JoinOmegaGroup(const struct DjiC6xxGroupTxConfig& tx_cfg);

    [[deprecated("Split into Init() + JoinOmegaGroup().")]]
    void InitAndJoinOmegaGroup(BspCanHandle can, const Config& cfg, const struct DjiC6xxGroupTxConfig& tx_cfg) {
        Init(can, cfg);
        SetTargetOmega(0.0f);
        JoinOmegaGroup(tx_cfg);
    }

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    void SetTargetOmega(float omega);
    void SetTargetCurrent(float current);

    void Update();

    // DJI 组帧使用的 raw 电流（16-bit signed, big-endian per slot）
    int16_t target_current_raw() const;

    float now_angle_rad() const { return now_angle_; }
    float now_omega_out_rad_s() const { return now_omega_out_; }
    float now_current_a() const { return now_current_; }
    float temperature_c() const { return temperature_; }

    float target_current_a() const { return target_current_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint16_t rx_std_id() const { return cfg_.rx_std_id; }

private:
    static constexpr float k2pi = 6.283185307179586f;

    Config cfg_{};
    BspCanHandle can_ = nullptr;

    uint16_t last_enc_ = 0;
    int32_t total_round_ = 0;
    float now_angle_ = 0.0f;
    float now_omega_out_ = 0.0f;
    float now_current_ = 0.0f;
    float temperature_ = 0.0f;

    float target_omega_out_ = 0.0f;
    float target_current_ = 0.0f;

    alg::Pid pid_omega_{};
};

// ===== RTOS task (driver-level wiring) =====
// 通用链路：
//   orb::dji_c6xx_omega_cmd (按 rx_std_id 定位) -> PID Update -> 0x200/0x1FF/0x2FF 组帧 -> orb::can_tx
struct DjiC6xxGroupTxConfig {
    orb::CanBus bus = orb::CanBus::CAN1;
    uint16_t group_std_id = 0x200;
};

} // namespace actuator::drivers

namespace actuator::instances {

// ===== Global DJI motor instances =====
// 定义与实现见：Device/actuator/drivers/dji_c6xx_min.cpp

extern actuator::drivers::DjiC6xxMin dji_201;
extern actuator::drivers::DjiC6xxMin dji_202;
extern actuator::drivers::DjiC6xxMin dji_203;
extern actuator::drivers::DjiC6xxMin dji_204;

} // namespace actuator::instances
