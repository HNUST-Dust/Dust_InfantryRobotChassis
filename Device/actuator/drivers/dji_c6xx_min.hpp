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

    void OnRx(const uint8_t data[8]);

    void SetTargetOmega(float omega_out_rad_s);
    void SetTargetCurrent(float current_a);

    void Update();

    // DJI 组帧使用的 raw 电流（16-bit signed, big-endian per slot）
    int16_t target_current_raw() const;

    float now_angle_rad() const { return now_angle_rad_; }
    float now_omega_out_rad_s() const { return now_omega_out_rad_s_; }
    float now_current_a() const { return now_current_a_; }
    float temperature_c() const { return temperature_c_; }

    float target_current_a() const { return target_current_a_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint16_t rx_std_id() const { return cfg_.rx_std_id; }

private:
    static constexpr float k2pi = 6.283185307179586f;

    Config cfg_{};
    BspCanHandle can_ = nullptr;

    uint16_t last_enc_ = 0;
    int32_t total_round_ = 0;
    float now_angle_rad_ = 0.0f;
    float now_omega_out_rad_s_ = 0.0f;
    float now_current_a_ = 0.0f;
    float temperature_c_ = 0.0f;

    float target_omega_out_rad_s_ = 0.0f;
    float target_current_a_ = 0.0f;

    alg::Pid pid_omega_{};
};

} // namespace actuator::drivers
