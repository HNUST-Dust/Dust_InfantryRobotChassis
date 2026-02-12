#pragma once

#include <cstdint>

#include "bsp_can_port.h"
#include "control/alg_pid.h"
#include "can_topics.hpp"

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

    // 注册该电机到共享运行时线程，并参与“组电流”组帧发送。
    // - tx_id: 组帧发送 ID，DJI C6xx 默认为 0x200
    // - slot: 由 (rx_id - (tx_id + 1)) 推导，范围 [0..3]
    void JoinRuntime(uint16_t tx_id = 0x200);

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    void SetTargetOmega(float omega);
    void SetTargetCurrent(float current);

    void Update();

    // DJI 组帧使用的 raw 电流（16-bit signed, big-endian per slot）
    int16_t target_current_raw() const;

    float now_angle_rad() const { return now_angle_; }
    float now_omega_out_rad_s() const { return now_omega_out_; }
    // 统一命名：now_omega_rad_s 表示“输出轴角速度”（与 now_omega_out_rad_s 含义一致）
    float now_omega_rad_s() const { return now_omega_out_; }
    float now_current_a() const { return now_current_; }
    float temperature_c() const { return temperature_; }

    float target_current_a() const { return target_current_; }
    float target_omega_rad_s() const { return target_omega_out_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint16_t rx_id() const { return cfg_.rx_std_id; }
    uint16_t tx_id() const { return joined_tx_id_; }

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

    // JoinRuntime 时记录的组帧发送 ID（用于统一 tx_id 命名）。
    uint16_t joined_tx_id_ = 0;

    alg::Pid pid_omega_{};
};

// ===== RTOS task (driver-level wiring) =====
// Runtime thread (driver-level wiring):
//   orb::dji_c6xx_omega_cmd (按 rx_id 定位) -> PID Update -> group frame(tx_id) -> orb::can_tx

} // namespace actuator::drivers

namespace actuator::instances {

// ===== Global DJI motor instances =====
// 定义与实现见：Device/motors/dji_c6xx_min.cpp

extern actuator::drivers::DjiC6xxMin dji_201;
extern actuator::drivers::DjiC6xxMin dji_202;
extern actuator::drivers::DjiC6xxMin dji_203;
extern actuator::drivers::DjiC6xxMin dji_204;

} // namespace actuator::instances
