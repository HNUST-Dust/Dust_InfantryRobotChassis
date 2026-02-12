#pragma once

#include <cstdint>

#include "../../../Platform/bsp_can_port.h"
#include "../../../communication_topic/can_topics.hpp"

namespace actuator::drivers {

class DmMitMin final {
public:
    struct Config {
        orb::CanBus bus = orb::CanBus::CAN1;
        uint8_t can_rx_id = 0x01;
        uint8_t master_id = 0x01;
        uint16_t base_std_id = 0x00;

        float angle_max = 12.5f;
        float omega_max = 45.0f;
        float torque_max = 10.0f;
    };

    void Init(BspCanHandle can, const Config& cfg);

    // Registers this motor into the DM MIT control runtime.
    // After JoinRuntime(), motor-level topics (orb::dm_mit_target_cmd / orb::dm_mit_admin_cmd)
    // will be dispatched to this instance by (bus + can_rx_id).
    void JoinRuntime();

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    void SetControl(float angle, float omega, float torque);

    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    // Bring-up helper (blocking, best-effort).
    // Uses DWT delay internally so it can run before osKernelStart().
    void BringUpDefault();

    void Enter();
    void Exit();
    void ClearError();
    void SaveZero();

    float now_angle() const { return now_angle_; }
    float now_omega() const { return now_omega_; }
    float now_torque() const { return now_torque_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint8_t can_rx_id() const { return cfg_.can_rx_id; }

private:
    Config cfg_{};
    BspCanHandle can_ = nullptr;
    bool joined_runtime_ = false;

    float now_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;
    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
    void PublishAdminTail(uint8_t tail);
};
} // namespace actuator::drivers

namespace actuator::instances {

// ===== Global DM motor instances =====
// 定义与实现见：Device/actuator/drivers/dm_mit_min.cpp

extern actuator::drivers::DmMitMin dm_01;
extern actuator::drivers::DmMitMin dm_02;

} // namespace actuator::instances
