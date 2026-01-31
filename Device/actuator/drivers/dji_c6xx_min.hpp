#pragma once

#include <cstdint>

#include "../../../Platform/bsp_can_port.h"
#include "../../../Algorithm/alg_pid.h"
#include "../../../communication_topic/motor_topics.hpp"

#include "../actuator_iface.hpp"

namespace actuator::drivers {

class DjiC6xxMin final : public actuator::IActuator {
public:
    enum class ControlMethod : uint8_t {
        Current = 0,
        Omega,
    };

    struct Config {
        orb::MotorBus bus = orb::MotorBus::CAN1;
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

    // publish to orb::dji_current_group_cmd
    void PublishTxTopic();

    float now_angle_rad() const { return now_angle_rad_; }
    float now_omega_out_rad_s() const { return now_omega_out_rad_s_; }
    float now_current_a() const { return now_current_a_; }
    float temperature_c() const { return temperature_c_; }

    float target_current_a() const { return target_current_a_; }

    // ===== IActuator =====
    void Bind(BspCanHandle can) override { can_ = can; }
    void OnRx(const BspCanFrame* frame) override { OnRx(frame->data); }
    void SetCmd(const actuator::Cmd& cmd) override;
    void Update(float /*dt_s*/) override { Update(); }
    void PublishTx() override { PublishTxTopic(); }
    const actuator::State& GetState() const override { return st_; }

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

    Pid pid_omega_{};

    actuator::Cmd cmd_{};
    actuator::State st_{};
};

} // namespace actuator::drivers
