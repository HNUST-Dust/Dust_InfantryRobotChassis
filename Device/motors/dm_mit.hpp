#pragma once

#include <cstdint>

#include "bsp_can_port.h"
#include "control/alg_pid.h"
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

    // ===== Cascaded PID (Angle -> Omega -> Torque) =====
    // 说明：
    // - PID 放在电机类内（与 DjiC6xxMin 风格一致）。
    // - 反馈可由上层传入（例如云台 IMU 模式下使用 IMU 角度/角速度）。
    // - 输出为 torque（N·m），用于 MIT 协议的 t 字段。

    void ConfigureCascadePid(const alg::PidConfig& angle_cfg,
                             const alg::PidConfig& omega_cfg,
                             float angle_to_omega_sign,
                             float omega_feedback_sign);

    enum class CascadeAxis : uint8_t {
        Yaw = 0,
        Pitch,
        Unknown = 255,
    };

    enum class CascadeFeedback : uint8_t {
        Imu = 0,
        Encoder,
    };

    // 使用工程内“云台默认参数”配置串级 PID。
    // 说明：这些参数原本位于 app_gimbal.cpp，现在下沉到电机类中统一管理。
    void ConfigureGimbalCascadePidDefaults(CascadeAxis axis, CascadeFeedback fb);

    CascadeAxis cascade_axis() const { return cascade_axis_; }

    enum class CascadeMode : uint8_t {
        Angle = 0,
        Omega,
    };

    // 单步更新：根据模式 + 目标 + 测量值，内部完成 Angle->Omega->Torque 计算。
    // - mode==Angle: target_angle 生效；target_omega/omega_ff 仅用于 Omega 模式
    // - mode==Omega: omega_sp = target_omega + omega_ff
    // 返回：torque_cmd (N·m)
    float UpdateCascadeTorque(CascadeMode mode,
                              float target_angle_rad,
                              float target_omega_rad_s,
                              float omega_ff_rad_s,
                              float measured_angle_rad,
                              float measured_omega_rad_s,
                              float* out_omega_sp_rad_s = nullptr);

    // 角度环输出：目标角度 -> 目标角速度（rad/s）
    float UpdateAngleToOmega(float target_angle_rad, float measured_angle_rad);

    // 速度环输出：目标角速度 -> 目标扭矩（N·m）
    float UpdateOmegaToTorque(float target_omega_rad_s, float measured_omega_rad_s);

    // 级联：角度模式一次算出扭矩
    float UpdateCascadeTorqueAngleMode(float target_angle_rad,
                                       float measured_angle_rad,
                                       float measured_omega_rad_s);

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
    float now_total_angle_rad() const { return now_total_angle_; }
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
    float now_total_angle_ = 0.0f;
    float now_omega_ = 0.0f;
    float now_torque_ = 0.0f;

    bool raw_angle_inited_ = false;
    float last_raw_angle_ = 0.0f;

    float ctrl_angle_ = 0.0f;
    float ctrl_omega_ = 0.0f;
    float ctrl_torque_ = 0.0f;

    bool cascade_pid_inited_ = false;
    float angle_to_omega_sign_ = 1.0f;
    float omega_feedback_sign_ = 1.0f;
    float last_torque_cmd_ = 0.0f;

    CascadeAxis cascade_axis_ = CascadeAxis::Unknown;

    alg::Pid pid_angle_{};
    alg::Pid pid_omega_{};

    static void PackMit(float p, float v, float kp, float kd, float t, uint8_t out[8],
                        float pmax, float vmax, float kpmax, float kdmax, float tmax);

    void PublishFrame(uint16_t std_id, const uint8_t data[8], uint8_t len);
    void PublishAdminTail(uint8_t tail);
};
} // namespace actuator::drivers

namespace actuator::instances {

extern actuator::drivers::DmMitMin dm_01;
extern actuator::drivers::DmMitMin dm_02;

} // namespace actuator::instances
