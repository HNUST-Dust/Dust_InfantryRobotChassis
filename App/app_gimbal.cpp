/**
 * @file app_gimbal.cpp
 * @brief Gimbal 实现：订阅 IMU/遥控/自瞄输入，完成融合与控制，并发布云台目标 Topic。
 *
 * 说明：
 * - 业务层只发布电机级目标（`orb::dm_mit_cascade_cmd` / `orb::dm_mit_admin_cmd`；Exit 时会发布一次 `orb::dm_mit_target_cmd` 置零扭矩），不直接发送 CAN。
 * - 电机反馈与 CAN 报文由 Device/actuator 运行时模块处理（CAN TX 仍通过 Topic：`orb::can_tx`）。
 */
#include "app_gimbal.h"

#include "../communication_topic/actuator_cmd_topics.hpp"
#include "../communication_topic/actuator_state_topics.hpp"
#include "../communication_topic/gimbal_state_topics.hpp"
#include "../communication_topic/mcu_topics.hpp"
#include "../Device/motor_ids.hpp"

#include "cmsis_os2.h"
extern "C" {
#include "FreeRTOS.h" 
#include "task.h"
}
#include <cstring>

#include "../Device/motors/dm_mit.hpp"

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

[[maybe_unused]] static constexpr TickType_t kFreeRtosTick0 = static_cast<TickType_t>(0);



namespace {
inline uint32_t now_ms()
{
    const uint32_t freq = osKernelGetTickFreq();
    if (freq == 0u) {
        return 0u;
    }
    return static_cast<uint32_t>(osKernelGetTickCount() * 1000u / freq);
}

} // namespace

Gimbal& Gimbal::Instance()
{
    static Gimbal inst;
    return inst;
}

void Gimbal::Init()
{
    if (started_) {
        configASSERT(false);
        return;
    }

    started_ = true;

    // 业务层不再持有串级 PID；电机类内部负责 PID 参数与串级计算。
#if defined(YAW_ENCODER_MODE)
    actuator::instances::dm_01.ConfigureGimbalCascadePidDefaults(
        actuator::drivers::DmMitMin::CascadeAxis::Yaw,
        actuator::drivers::DmMitMin::CascadeFeedback::Encoder);
#elif defined(YAW_IMU_MODE)
    actuator::instances::dm_01.ConfigureGimbalCascadePidDefaults(
        actuator::drivers::DmMitMin::CascadeAxis::Yaw,
        actuator::drivers::DmMitMin::CascadeFeedback::Imu);
#endif

#if defined(PITCH_ENCODER_MODE)
    actuator::instances::dm_02.ConfigureGimbalCascadePidDefaults(
        actuator::drivers::DmMitMin::CascadeAxis::Pitch,
        actuator::drivers::DmMitMin::CascadeFeedback::Encoder);
#elif defined(PITCH_IMU_MODE)
    actuator::instances::dm_02.ConfigureGimbalCascadePidDefaults(
        actuator::drivers::DmMitMin::CascadeAxis::Pitch,
        actuator::drivers::DmMitMin::CascadeFeedback::Imu);
#endif

    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void Gimbal::Exit()
{
    // 禁用串级控制（由 dm_mit 运行时线程消费并停止周期输出）

    orb::DmMitCascadeCmd c{};
    c.bus = orb::CanBus::CAN3;
    c.enable = false;

    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    orb::dm_mit_cascade_cmd.publish(c);

    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
    orb::dm_mit_cascade_cmd.publish(c);


    orb::DmMitTargetCmd t{};
    t.bus = orb::CanBus::CAN3;
    t.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    t.angle = 0.0f;
    t.omega = 0.0f;
    t.torque = 0.0f;
    orb::dm_mit_target_cmd.publish(t);
    
    t.bus = orb::CanBus::CAN3;
    t.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
    t.angle = 0.0f;
    t.omega = 0.0f;
    t.torque = 0.0f;
    orb::dm_mit_target_cmd.publish(t);
}

void Gimbal::SetYawZero()
{
    orb::DmMitAdminCmd cmd{};
    cmd.bus = orb::CanBus::CAN3;
    cmd.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    cmd.op = orb::DmMitAdminOp::SaveZero;
    orb::dm_mit_admin_cmd.publish(cmd);
}

void Gimbal::Output()
{
    orb::DmMitCascadeCmd c{};
    c.bus = orb::CanBus::CAN3;
    c.enable = true;

    // Yaw: 角度/角速度模式由上层决定
    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    c.mode = (yaw_control_type_ == GIMBAL_CONTROL_TYPE_OMEGA) ? orb::DmMitCascadeMode::Omega
                                                              : orb::DmMitCascadeMode::Angle;
    c.target_angle = virtual_yaw_angle_;
    c.target_omega = GetTargetYawOmega();
    c.omega_ff = GetYawOmegaFeedforword();
    orb::dm_mit_cascade_cmd.publish(c);

    // Pitch: 当前仅角度模式
    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
    c.mode = orb::DmMitCascadeMode::Angle;
    c.target_angle = virtual_pitch_angle_;
    c.target_omega = 0.0f;
    c.omega_ff = 0.0f;
    orb::dm_mit_cascade_cmd.publish(c);
}

void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);
    self->Task();  
}

void Gimbal::Task()
{
    static constexpr float kYawSensitivityUsedImu = 0.00800f;
    static constexpr float kYawSpeedSensitivity = 0.05f;
    static constexpr float kPitchRangeMaxUseImu = 15.0f;
    static constexpr float kChassisSpinSpeed = 30.0f;
    static constexpr float kYawFeedforwardRatio = 0.19f;

    Subscription<orb::McuControl> mcu_control_sub(orb::mcu_control);
    Subscription<orb::McuAutoAim> mcu_autoaim_sub(orb::mcu_autoaim);
    Subscription<orb::McuImu> mcu_imu_sub(orb::mcu_imu);

    static constexpr uint8_t kYawCanRxId = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    static constexpr uint8_t kPitchCanRxId = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
    Subscription<orb::DmMitFeedback> yaw_fb_sub(
        orb::dm_mit_feedback[orb::dm_mit_feedback_index(orb::CanBus::CAN3, kYawCanRxId)]);
    Subscription<orb::DmMitFeedback> pitch_fb_sub(
        orb::dm_mit_feedback[orb::dm_mit_feedback_index(orb::CanBus::CAN3, kPitchCanRxId)]);

    orb::DmMitFeedback yaw_fb{};
    orb::DmMitFeedback pitch_fb{};
    orb::McuControl mcu{};
    orb::McuAutoAim autoaim{};
    orb::McuImu imu{};

    // 3s 等待陀螺仪收敛
    const uint32_t t0 = now_ms();
    while ((now_ms() - t0) < 3000u) {
        (void)mcu_control_sub.copy(mcu);
        (void)mcu_autoaim_sub.copy(autoaim);
        (void)mcu_imu_sub.copy(imu);
        osDelay(1);
    }

    if (!minipc_filters_inited_) {
        minipc_yaw_recive_filter_.configure(20.0f, 0.001f);
        minipc_pitch_recive_filter_.configure(20.0f, 0.001f);
        minipc_filters_inited_ = true;
    }

    for (;;)
    {
        (void)mcu_control_sub.copy(mcu);
        (void)mcu_autoaim_sub.copy(autoaim);
        (void)mcu_imu_sub.copy(imu);

        // 电机反馈（用于 now_* 状态/发布）
        if (yaw_fb_sub.copy(yaw_fb)) {
            now_yaw_angle_ = yaw_fb.total_angle_rad;
            yaw_now_angle_noncumulative_ = yaw_fb.angle_rad;
            now_yaw_omega_ = yaw_fb.omega_rad_s;
        }
        if (pitch_fb_sub.copy(pitch_fb)) {
            now_pitch_angle_ = pitch_fb.total_angle_rad;
            pitch_now_angle_noncumulative_ = pitch_fb.angle_rad;
            now_pitch_omega_ = pitch_fb.omega_rad_s;
        }

        // IMU 输入
        SetYawImuAngle(imu.yaw_total_angle_f);
        SetYawImuOmega(imu.yaw_omega_f);
        SetPitchImuAngle(imu.pitch_f);
        SetPitchImuOmega(imu.pitch_omega_f);

        // miniPC 自瞄融合
        if (mcu.auto_aim_flag == 1) {
            float pitch = imu.pitch_f - autoaim.pitch_angle * RAD_TO_DEG;
            minipc_pitch_recive_filter_.update(pitch);
            pitch = minipc_pitch_recive_filter_.value();
            virtual_pitch_angle_ = pitch;

            float yaw = imu.yaw_total_angle_f + (autoaim.yaw_angle * RAD_TO_DEG);
            minipc_yaw_recive_filter_.update(yaw);
            yaw = minipc_yaw_recive_filter_.value();
            virtual_yaw_angle_ = yaw;
        }

        // 手动输入融合
        virtual_yaw_angle_ += (127.0f - mcu.yaw) * kYawSensitivityUsedImu;
        if (mcu.auto_aim_flag == 0) {
            virtual_pitch_angle_ = (127.0f - mcu.pitch_angle) * (2.0f * kPitchRangeMaxUseImu / 128.0f);
        }
        if (virtual_pitch_angle_ >= 1.5f * kPitchRangeMaxUseImu) {
            virtual_pitch_angle_ = 1.5f * kPitchRangeMaxUseImu;
        } else if (virtual_pitch_angle_ <= (-3.0f * kPitchRangeMaxUseImu)) {
            virtual_pitch_angle_ = (-3.0f * kPitchRangeMaxUseImu);
        }

        SetVirtualYawAngle(virtual_yaw_angle_);
        SetVirtualPitchAngle(virtual_pitch_angle_);

        // 默认：角度模式
        SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_ANGLE);
        SetYawOmegaFeedforword(0.0f);
        SetTargetYawOmega(0.0f);

        // 模式：小陀螺 / 疯车保护
        switch (mcu.chassis_spin)
        {
            case orb::McuChassisSpinMode::Clockwise:
                SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_OMEGA);
                SetYawOmegaFeedforword(kYawFeedforwardRatio * kChassisSpinSpeed);
                SetTargetYawOmega((mcu.yaw - 127.0f) * kYawSpeedSensitivity);
                break;
            case orb::McuChassisSpinMode::Disable:
                break;
            case orb::McuChassisSpinMode::CounterClockwise:
                Exit();
                break;
            default:
                break;
        }

        // 命令：置零/恢复
        if (mcu.reset_zero == 1) {
            SetYawZero();

            orb::DmMitAdminCmd c{};
            c.bus = orb::CanBus::CAN3;

            c.op = orb::DmMitAdminOp::ClearError;
            c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
            orb::dm_mit_admin_cmd.publish(c);
            c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
            orb::dm_mit_admin_cmd.publish(c);

            c.op = orb::DmMitAdminOp::Enter;
            c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
            orb::dm_mit_admin_cmd.publish(c);
            c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
            orb::dm_mit_admin_cmd.publish(c);
        }

        // 发布 gimbal 状态（供其他模块订阅读取）
        orb::GimbalState st{};
        st.yaw_angle = GetNowYawAngle();
        st.yaw_omega = GetNowYawOmega();
        st.pitch_angle = GetNowPitchAngle();
        st.pitch_omega = GetNowPitchOmega();
        st.yaw_angle_noncumulative = GetYawNowAngleNoncumulative();
        st.pitch_angle_noncumulative = GetPitchNowAngleNoncumulative();
        orb::gimbal_state.publish(st);

        // 发布给外部 MCU 的云台信息（由 McuComm::TxTask 订阅并转发到 CAN）
        orb::GimbalInfoTx tx{};
        tx.yaw_angle = st.yaw_angle;
        tx.yaw_omega = st.yaw_omega;
        tx.pitch_angle = st.pitch_angle;
        tx.pitch_omega = st.pitch_omega;
        orb::gimbal_info_tx.publish(tx);

        Output();

        osDelay(1); // 1khz电机控制频率
    }
}

