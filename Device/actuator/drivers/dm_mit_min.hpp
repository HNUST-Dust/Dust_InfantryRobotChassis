#pragma once

#include <cstdint>

#include "../../../Platform/bsp_can_port.h"
#include "../../../communication_topic/dm_motor_topics.hpp"

#include "../actuator_iface.hpp"

namespace actuator::drivers {

class DmMitMin final : public actuator::IActuator {
public:
    struct Config {
        orb::DmBus bus = orb::DmBus::CAN1;
        uint8_t can_rx_id = 0x01;
        uint8_t master_id = 0x01;
        uint16_t base_std_id = 0x00;

        float angle_max = 12.5f;
        float omega_max = 45.0f;
        float torque_max = 10.0f;
    };

    void Init(BspCanHandle can, const Config& cfg);

    void OnRx(const uint8_t data[8]);

    void SetControl(float angle, float omega, float torque);

    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    void Enter();
    void Exit();
    void ClearError();
    void SaveZero();

    float now_angle() const { return now_angle_; }
    float now_omega() const { return now_omega_; }
    float now_torque() const { return now_torque_; }

    // ===== IActuator =====
    void Bind(BspCanHandle can) override { can_ = can; }
    void OnRx(const BspCanFrame* frame) override { OnRx(frame->data); }
    void SetCmd(const actuator::Cmd& cmd) override;
    void Update(float /*dt_s*/) override {}
    void PublishTx() override { PublishMitTx(); }
    const actuator::State& GetState() const override { return st_; }

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;

    float now_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    actuator::Cmd cmd_{};
    actuator::State st_{};

    static uint16_t MitTxStdId(uint8_t master_id);
    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
};

} // namespace actuator::drivers
