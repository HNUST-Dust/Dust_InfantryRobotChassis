#include "dji_c6xx_min.hpp"

#include <cmath>
#include <cstring>

namespace actuator::drivers {

namespace {
inline uint16_t u16_be(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]);
}
inline int16_t i16_be(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}
inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
} // namespace

void DjiC6xxMin::Init(BspCanHandle can, const Config& cfg) {
    can_ = can;
    cfg_ = cfg;

    pid_omega_.Init(cfg_.kp, cfg_.ki, cfg_.kd);

    last_enc_ = 0;
    total_round_ = 0;
    now_angle_rad_ = 0.0f;
    now_omega_out_rad_s_ = 0.0f;
    now_current_a_ = 0.0f;
    temperature_c_ = 0.0f;

    target_omega_out_rad_s_ = 0.0f;
    target_current_a_ = 0.0f;

    st_ = actuator::State{};
}

void DjiC6xxMin::OnRx(const uint8_t data[8]) {
    const uint16_t enc = u16_be(&data[0]);
    const int16_t omega_rpm = i16_be(&data[2]);
    const int16_t current_raw = i16_be(&data[4]);
    const uint8_t temp = data[6];

    // unwrap encoder (same idea as legacy driver, but minimal)
    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(last_enc_);
        if (diff > 4096) {
            total_round_ -= 1;
        } else if (diff < -4096) {
            total_round_ += 1;
        }
    }
    last_enc_ = enc;

    const int32_t total_enc = total_round_ * static_cast<int32_t>(cfg_.enc_per_round) + static_cast<int32_t>(enc);
    const float motor_angle_rad = (static_cast<float>(total_enc) / static_cast<float>(cfg_.enc_per_round)) * k2pi;
    now_angle_rad_ = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle_rad / cfg_.gearbox_ratio) : motor_angle_rad;

    // omega: rpm -> rad/s (motor side) then / gearbox_ratio to output side
    const float omega_motor_rad_s = (static_cast<float>(omega_rpm) * k2pi) / 60.0f;
    now_omega_out_rad_s_ = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor_rad_s / cfg_.gearbox_ratio) : omega_motor_rad_s;

    // current: legacy uses 16384/20 scale; keep raw->A mapping consistent
    now_current_a_ = static_cast<float>(current_raw) * (20.0f / 16384.0f);
    temperature_c_ = static_cast<float>(temp);

    // mirror into State
    st_.angle = now_angle_rad_;
    st_.omega = now_omega_out_rad_s_;
    st_.current = now_current_a_;
    st_.temperature = temperature_c_;
    st_.online = true;
}

void DjiC6xxMin::SetCmd(const actuator::Cmd& cmd) {
    cmd_ = cmd;
    switch (cmd.mode) {
    case actuator::Mode::Omega:
        SetTargetOmega(cmd.target_omega);
        break;
    case actuator::Mode::Current:
    case actuator::Mode::Torque:
        SetTargetCurrent(cmd.target_current != 0.0f ? cmd.target_current : cmd.target_torque);
        break;
    default:
        SetTargetOmega(0.0f);
        break;
    }
}

void DjiC6xxMin::SetTargetOmega(float omega_out_rad_s) {
    target_omega_out_rad_s_ = omega_out_rad_s;
    cfg_.method = ControlMethod::Omega;
}

void DjiC6xxMin::SetTargetCurrent(float current_a) {
    target_current_a_ = current_a;
    cfg_.method = ControlMethod::Current;
}

void DjiC6xxMin::Update() {
    if (cfg_.method == ControlMethod::Omega) {
        pid_omega_.SetTarget(target_omega_out_rad_s_);
        pid_omega_.SetNow(now_omega_out_rad_s_);
        pid_omega_.CalculatePeriodElapsedCallback();
        target_current_a_ = pid_omega_.GetOut();
    }

    target_current_a_ = clampf(target_current_a_, -cfg_.current_limit, cfg_.current_limit);
}

void DjiC6xxMin::PublishTxTopic() {
    if (!can_) {
        return;
    }

    // map motor std_id -> group std_id + index
    // 0x201..0x204 -> 0x200 [0..3]
    // 0x205..0x208 -> 0x1FF [0..3]
    const uint16_t id = cfg_.rx_std_id;
    uint16_t group_id = 0;
    uint8_t idx = 0;

    if (id >= 0x201 && id <= 0x204) {
        group_id = 0x200;
        idx = static_cast<uint8_t>(id - 0x201);
    } else if (id >= 0x205 && id <= 0x208) {
        group_id = 0x1FF;
        idx = static_cast<uint8_t>(id - 0x205);
    } else {
        // out of supported range
        return;
    }

    // A -> raw current for DJI group frame
    const int16_t raw = static_cast<int16_t>(clampf(target_current_a_, -cfg_.current_limit, cfg_.current_limit) * (16384.0f / 20.0f));

    orb::DjiCurrentGroupCmd cmd{};
    cmd.bus = cfg_.bus;
    cmd.std_id = group_id;
    cmd.current[0] = 0;
    cmd.current[1] = 0;
    cmd.current[2] = 0;
    cmd.current[3] = 0;
    cmd.current[idx] = raw;

    orb::dji_current_group_cmd.publish(cmd);
}

} // namespace actuator::drivers
