/**
 * @file app_gimbal.cpp
 * @brief Gimbal 实现：订阅 IMU/遥控/自瞄输入，完成融合与控制，并发布云台目标 Topic。
 *
 * 说明：
 * - 业务层只发布目标（`orb::gimbal_dm_target` / `orb::gimbal_dm_admin_cmd`），不直接发送 CAN。
 * - 电机反馈与 CAN 报文由 Device/MotorActuatorTask 处理。
 * - 守护（daemon_supervisor）仅在“收到新外部数据”时 feed，避免任务空转误判在线。
 */
#include "app_gimbal.h"
#include "cmsis_os2.h"

extern "C" {
#include "FreeRTOS.h" // NOLINT(misc-include-cleaner)
#include "task.h"
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

[[maybe_unused]] static constexpr TickType_t kFreeRtosTick0 = static_cast<TickType_t>(0);

#include "../communication_topic/app_motor_topics.hpp"

#include "../communication_topic/gimbal_state_topics.hpp"
#include "../communication_topic/mcu_topics.hpp"

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

void gimbal_daemon_fault(DaemonClient& c)
{
    auto* self = static_cast<Gimbal*>(c.owner());
    if (self) {
        self->Exit();
    }
}

DaemonClient* s_gimbal_daemon = nullptr;
} // namespace

Gimbal& Gimbal_Instance()
{
    static Gimbal inst;
    return inst;
}

void Gimbal::Start()
{
    if (started_) {
        configASSERT(false);
        return;
    }

    // Register daemon client (FATAL + CRITICAL)
    {
        static DaemonClient s_daemon(
            200,
            gimbal_daemon_fault,
            this,
            DaemonClient::Domain::CONTROL,
            DaemonClient::FaultLevel::FATAL,
            DaemonClient::Priority::CRITICAL);
        s_gimbal_daemon = &s_daemon;
        (void)DaemonSupervisor::register_client(s_gimbal_daemon);
        s_gimbal_daemon->feed(now_ms());
    }

    started_ = true;

    // 业务层只做控制参数初始化（PID/滤波器等）

    const auto make_pid_cfg = [](float kp,
                                 float ki,
                                 float kd,
                                 float kf,
                                 float i_out_max,
                                 float out_max,
                                 float dt,
                                 float dead_zone,
                                 float i_variable_speed_A,
                                 float i_variable_speed_B,
                                 float i_separate_threshold,
                                 alg::DFirst d_first,
                                 float d_lpf_tau) {
        alg::PidConfig cfg{};
        cfg.kp = kp;
        cfg.ki = ki;
        cfg.kd = kd;
        cfg.kf = kf;
        cfg.i_out_max = i_out_max;
        cfg.out_max = out_max;
        cfg.dt = dt;
        cfg.dead_zone = dead_zone;
        cfg.i_variable_speed_A = i_variable_speed_A;
        cfg.i_variable_speed_B = i_variable_speed_B;
        cfg.i_separate_threshold = i_separate_threshold;
        cfg.d_first = d_first;
        cfg.d_lpf_tau = d_lpf_tau;
        return cfg;
    };

#ifdef YAW_ENCODER_MODE
     //yaw轴角度环PID初始化
    yaw_angle_pid_.configure(make_pid_cfg(
        100.0f,
        5.0f,
        15.0f,
        0.0f,
        0.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.01f));
#endif
#ifdef YAW_IMU_MODE
     //yaw轴角度环PID初始化
    yaw_angle_pid_.configure(make_pid_cfg(
        1.8f,
        0.1f,
        0.4f,
        1.0f,
        44.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.01f));
#endif
#ifdef PITCH_ENCODER_MODE
    //pitch轴角度环PID初始化
    pitch_angle_pid_.configure(make_pid_cfg(
        350.0f,//350
        20.0f,//160
        20.0f,
        0.0f,
        0.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.02f));
#endif
#ifdef PITCH_IMU_MODE
    //pitch轴角度环PID初始化
    pitch_angle_pid_.configure(make_pid_cfg(
        4.0f,
        1.50f,
        0.25f,
        0.0f,
        44.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.02f));
#endif
    //yaw轴速度环PID初始化
    yaw_omega_pid_.configure(make_pid_cfg(
        0.04f,
        0.008f,
        0.00015f,
        0.1f,
        3.0f,
        9.9f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.01f));
    //pitch轴速度环PID初始化
    pitch_omega_pid_.configure(make_pid_cfg(
        0.08f,
        0.008f,
        0.0000f,
        1.0f,
        3.0f,
        9.9f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        alg::DFirst::Disable,
        0.01f));
    // yaw轴速度环低通滤波器初始化
    yaw_omega_filter_.configure(15.0f, 0.001f);
    pitch_omega_filter_.configure(15.0f, 0.001f);

    // 不再在这里 Init/Enter/Output 电机；交给 MotorActuatorTask

    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,//剩余260字节
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void Gimbal::Exit()
{
    // 方案B：让执行器层退出（急停）
    // orb::GimbalDmAdminCmd admin{};
    // admin.op = orb::GimbalDmAdminOp::BothExit;
    // orb::gimbal_dm_admin_cmd.publish(admin);

    // 同时将目标清零（由MotorActuatorTask输出到电机）
    // orb::GimbalDmTarget t{};
    // t.yaw_angle = 0.0f;
    // t.yaw_omega = 0.0f;
    // t.yaw_torque = 0.0f;
    // t.pitch_angle = 0.0f;
    // t.pitch_omega = 0.0f;
    // t.pitch_torque = 0.0f;
    // orb::gimbal_dm_target.publish(t);

    // 新方案：业务层只发布云台目标（不接触 CAN 信息）
    orb::GimbalDmTarget t{};
    t.yaw_angle = 0.0f;
    t.yaw_omega = 0.0f;
    t.yaw_torque = 0.0f;
    t.pitch_angle = 0.0f;
    t.pitch_omega = 0.0f;
    t.pitch_torque = 0.0f;
    orb::gimbal_dm_target.publish(t);
}

/**
 * @brief 自身解算
 *
 */
void Gimbal::SelfResolution()
{
    // 业务层不再直接读取电机反馈；这里的 now_* 只做状态镜像（来自 IMU）
    now_yaw_angle_ = imu_yaw_angle_;
    now_pitch_angle_ = imu_pitch_angle_;

    now_yaw_omega_ = imu_yaw_omega_;
    now_pitch_omega_ = imu_pitch_omega_;

    // 执行器层（MotorActuatorTask）会维护实际 torque 反馈；这里先置 0
    now_yaw_torque_ = 0.0f;
    now_pitch_torque_ = 0.0f;

    // yaw 角度环
#ifdef YAW_ENCODER_MODE
    float yaw_err = CalcYawError(virtual_yaw_angle_, normalize_angle_pm_pi(GetNowYawAngle()/0.8f));
    const float yaw_angle_out = yaw_angle_pid_.update(0.0f, yaw_err);
    if(yaw_control_type_ == GIMBAL_CONTROL_TYPE_ANGLE){
        SetTargetYawOmega(-yaw_angle_out);
    }else if(yaw_control_type_ == GIMBAL_CONTROL_TYPE_OMEGA){
        SetTargetYawOmega(GetTargetYawOmega() + GetYawOmegaFeedforword());
    }
#endif
#ifdef YAW_IMU_MODE
    if (yaw_control_type_ == GIMBAL_CONTROL_TYPE_ANGLE) {
        const float yaw_angle_out = yaw_angle_pid_.update(virtual_yaw_angle_, imu_yaw_angle_);
        SetTargetYawOmega(-yaw_angle_out);
    } else if (yaw_control_type_ == GIMBAL_CONTROL_TYPE_OMEGA) {
        SetTargetYawOmega(GetTargetYawOmega() + GetYawOmegaFeedforword());
    }
#endif

    // yaw 速度环
    (void)yaw_omega_pid_.update(GetTargetYawOmega(), -imu_yaw_omega_);

    // pitch 角度环
#ifdef PITCH_ENCODER_MODE
    const float pitch_angle_out = pitch_angle_pid_.update(virtual_pitch_angle_, GetPitchNowAngleNoncumulative());
    SetTargetPitchOmega(pitch_angle_out);
#endif
#ifdef PITCH_IMU_MODE
    const float pitch_angle_out = pitch_angle_pid_.update(virtual_pitch_angle_, imu_pitch_angle_);
    SetTargetPitchOmega(-pitch_angle_out);
#endif

    // pitch 速度环
    (void)pitch_omega_pid_.update(GetTargetPitchOmega(), imu_pitch_omega_);
}

void Gimbal::SetYawZero()
{
    orb::GimbalDmAdminCmd cmd{};
    cmd.op = orb::GimbalDmAdminOp::YawSaveZero;
    orb::gimbal_dm_admin_cmd.publish(cmd);
}

void Gimbal::Output()
{
    orb::GimbalDmTarget t{};
    t.yaw_angle = GetTargetYawAngle();
    t.yaw_omega = GetTargetYawOmega();
    t.yaw_torque = 0.0f;
    t.pitch_angle = GetTargetPitchAngle();
    t.pitch_omega = GetTargetPitchOmega();
    t.pitch_torque = 0.0f;
    orb::gimbal_dm_target.publish(t);
}

/**
 * @brief 电机就近转位
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // 方案B：业务层不直接访问电机。就近转位基于当前姿态（imu）/目标角，等价处理：将目标 yaw 归一化到与当前 yaw 最近。
    float tmp_delta_angle = fmodf(target_yaw_angle_ - now_yaw_angle_, TWO_PI);
    if (tmp_delta_angle > PI) {
        tmp_delta_angle -= TWO_PI;
    } else if (tmp_delta_angle < -PI) {
        tmp_delta_angle += TWO_PI;
    }
    target_yaw_angle_ = now_yaw_angle_ + tmp_delta_angle;

    // pitch 保持在机械限位约束即可（由上层设定，或在 SetTargetPitchAngle 中限幅）
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

    // Distributed: Gimbal subscribes to inputs directly (no Robot aggregator)
    Subscription<orb::McuControl> mcu_control_sub(orb::mcu_control);
    Subscription<orb::McuAutoAim> mcu_autoaim_sub(orb::mcu_autoaim);
    Subscription<orb::McuImu> mcu_imu_sub(orb::mcu_imu);
    orb::McuControl mcu{};
    orb::McuAutoAim autoaim{};
    orb::McuImu imu{};

    // 3s 等待陀螺仪收敛：仍然以“收到新外部数据”为准喂守护，避免启动阶段误判离线。
    {
        const uint32_t t0 = now_ms();
        while ((now_ms() - t0) < 3000u) {
            const bool got_mcu = mcu_control_sub.copy(mcu);
            const bool got_autoaim = mcu_autoaim_sub.copy(autoaim);
            const bool got_imu = mcu_imu_sub.copy(imu);
            if ((got_mcu || got_autoaim || got_imu) && s_gimbal_daemon) {
                s_gimbal_daemon->feed(now_ms());
            }
            osDelay(1);
        }
    }
    if (!minipc_filters_inited_) {
        minipc_yaw_recive_filter_.configure(20.0f, 0.001f);
        minipc_pitch_recive_filter_.configure(20.0f, 0.001f);
        minipc_filters_inited_ = true;
    }

    uint8_t first_run_flag = 0; // 用于标记是否是第一次运行

    for (;;)
    {
        const bool got_mcu = mcu_control_sub.copy(mcu);
        const bool got_autoaim = mcu_autoaim_sub.copy(autoaim);
        const bool got_imu = mcu_imu_sub.copy(imu);
        if ((got_mcu || got_autoaim || got_imu) && s_gimbal_daemon) {
            s_gimbal_daemon->feed(now_ms());
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

            orb::GimbalDmAdminCmd admin{};
            admin.op = orb::GimbalDmAdminOp::BothClearError;
            orb::gimbal_dm_admin_cmd.publish(admin);
            admin.op = orb::GimbalDmAdminOp::BothEnter;
            orb::gimbal_dm_admin_cmd.publish(admin);
        }

        if(first_run_flag == 0){ // 第一次运行到这里，pre_pitch_angle未初始化
            first_run_flag = 1;
            pre_pitch_angle_ = target_pitch_angle_;
        }
        SelfResolution();

        // 发布 gimbal 状态（供其他模块订阅读取）
        {
            orb::GimbalState st{};
            st.yaw_angle = GetNowYawAngle();
            st.yaw_omega = GetNowYawOmega();
            st.pitch_angle = GetNowPitchAngle();
            st.pitch_omega = GetNowPitchOmega();
            st.yaw_angle_noncumulative = GetYawNowAngleNoncumulative();
            st.pitch_angle_noncumulative = GetPitchNowAngleNoncumulative();
            orb::gimbal_state.publish(st);
        }

        Output();
        osDelay(1); // 1khz电机控制频率
        pre_pitch_angle_ = target_pitch_angle_;
    }
}