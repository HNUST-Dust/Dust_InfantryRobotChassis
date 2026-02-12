#pragma once

#include <cstdint>

#include "bsp_can_port.h"
#include "can_topics.hpp"

namespace actuator::drivers {

class DmMitMin final {
public:
    struct Config {
        orb::CanBus bus = orb::CanBus::CAN1;
        // 统一命名：rx_id（该电机的低 4bit ID）
        uint8_t can_rx_id = 0x01;
        uint8_t master_id = 0x01;
        // 统一命名：tx_id（MIT 协议发送 std_id 的基址，高位部分）
        uint16_t base_std_id = 0x00;

        float angle_max = 12.5f;
        float omega_max = 45.0f;
        float torque_max = 10.0f;
    };

    void Init(BspCanHandle can, const Config& cfg);

    // 注册该电机到 DM MIT 控制运行时线程。
    // JoinRuntime() 后，会由运行时线程消费 motor-level topics：
    // - orb::dm_mit_target_cmd
    // - orb::dm_mit_admin_cmd
    // 并按 (bus + can_rx_id) 派发到对应实例。
    void JoinRuntime();

    // CAN RX complete callback entry (called from bus-level ID dispatch)
    void CanRxCpltCallback(const BspCanFrame* frame);

    // 设置目标（角度/角速度/力矩）。
    // 统一命名：SetTarget 等价于历史的 SetControl。
    void SetTarget(float angle_rad, float omega_rad_s, float torque_nm);
    void SetControl(float angle, float omega, float torque);

    void PublishMitTx(float kp = 0.0f, float kd = 0.0f);

    // 上电 bring-up 辅助（阻塞、best-effort）。
    // 内部用 DWT delay，因此可在 osKernelStart() 前调用。
    void BringUpDefault();

    void Enter();
    void Exit();
    void ClearError();
    void SaveZero();

    // 反馈状态（单位：rad / rad/s / N·m）
    float now_angle() const { return now_angle_; }
    float now_omega() const { return now_omega_; }
    float now_torque() const { return now_torque_; }
    // 统一命名（带单位后缀）
    float now_angle_rad() const { return now_angle_; }
    float now_omega_rad_s() const { return now_omega_; }
    float now_torque_nm() const { return now_torque_; }

    // 目标状态（单位：rad / rad/s / N·m）
    float target_angle_rad() const { return ctrl_angle_; }
    float target_omega_rad_s() const { return ctrl_omega_; }
    float target_torque_nm() const { return ctrl_torque_; }

    orb::CanBus bus() const { return cfg_.bus; }
    uint8_t can_rx_id() const { return cfg_.can_rx_id; }
    // 统一命名：rx_id/tx_id（保留 can_rx_id/base_std_id 以兼容历史调用）
    uint16_t rx_id() const { return static_cast<uint16_t>(cfg_.can_rx_id); }
    uint16_t tx_id() const { return cfg_.base_std_id; }

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
// 定义与实现见：Device/motors/dm_mit_min.cpp

extern actuator::drivers::DmMitMin dm_01;
extern actuator::drivers::DmMitMin dm_02;

} // namespace actuator::instances
