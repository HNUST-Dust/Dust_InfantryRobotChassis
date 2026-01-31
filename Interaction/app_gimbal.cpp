/**
 * @file app_gimbal.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "app_gimbal.h"
#include "alg_pid.h"
#include "cmsis_os2.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h" // for taskDISABLE_INTERRUPTS used by configASSERT
}

// 满足 clang-tidy: "Included header FreeRTOS.h is not used directly"
static constexpr uint32_t kFreeRtosTickPeriodMs = portTICK_PERIOD_MS;

#include "../communication_topic/motor_topics.hpp" // per-motor cmd/state + admin

#include "alg_math.h"
#include "low_pass_filter.hpp"
#include "../communication_topic/gimbal_topics.hpp"
#include "../communication_topic/gimbal_state_topics.hpp"

void Gimbal::Start()
{
    if (started_) {
        configASSERT(false);
        return;
    }
    started_ = true;

    // 业务层只做控制参数初始化（PID/滤波器等）

#ifdef YAW_ENCODER_MODE
     //yaw轴角度环PID初始化
    yaw_angle_pid_.Init(
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
        PID_D_First_DISABLE,
        0.01f  
    );
#endif
#ifdef YAW_IMU_MODE
     //yaw轴角度环PID初始化
    yaw_angle_pid_.Init(
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
        PID_D_First_DISABLE,
        0.01f  
    );
#endif
#ifdef PITCH_ENCODER_MODE
    //pitch轴角度环PID初始化
    pitch_angle_pid_.Init(
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
        PID_D_First_DISABLE,
        0.02f  
    );
#endif
#ifdef PITCH_IMU_MODE
    //pitch轴角度环PID初始化
    pitch_angle_pid_.Init(
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
        PID_D_First_DISABLE,
        0.02f  
    );
#endif
    //yaw轴速度环PID初始化
    yaw_omega_pid_.Init(
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
        PID_D_First_DISABLE,
        0.01f
    );
    //pitch轴速度环PID初始化
    pitch_omega_pid_.Init(
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
        PID_D_First_DISABLE,
        0.01f
    );
    // yaw轴速度环低通滤波器初始化
    yaw_omega_filter_.Init(15.0f,0.001f);
    pitch_omega_filter_.Init(15.0f,0.001f);

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

    // 新方案：直接对电机发布禁用/清零
    // yaw (0x12 on CAN3)
    {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN3;
        c.id.std_id = 0x12;
        c.mode = orb::MotorCtrlMode::Angle;
        c.target_angle = 0.0f;
        c.target_omega = 0.0f;
        orb::motor_cmd.publish(c);
    }
    // pitch (0x11 on CAN3)
    {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN3;
        c.id.std_id = 0x11;
        c.mode = orb::MotorCtrlMode::Angle;
        c.target_angle = 0.0f;
        c.target_omega = 0.0f;
        orb::motor_cmd.publish(c);
    }
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
    yaw_angle_pid_.SetTarget(0);
    float yaw_err = CalcYawError(virtual_yaw_angle_, normalize_angle_pm_pi(GetNowYawAngle()/0.8f));
    yaw_angle_pid_.SetNow(yaw_err);
    yaw_angle_pid_.CalculatePeriodElapsedCallback();
    if(yaw_control_type_ == GIMBAL_CONTROL_TYPE_ANGLE){
        SetTargetYawOmega(-yaw_angle_pid_.GetOut());
    }else if(yaw_control_type_ == GIMBAL_CONTROL_TYPE_OMEGA){
        SetTargetYawOmega(GetTargetYawOmega() + GetYawOmegaFeedforword());
    }
#endif
#ifdef YAW_IMU_MODE
    yaw_angle_pid_.SetTarget(virtual_yaw_angle_);
    yaw_angle_pid_.SetNow(imu_yaw_angle_);
    yaw_angle_pid_.CalculatePeriodElapsedCallback();
    SetTargetYawOmega(-yaw_angle_pid_.GetOut());
#endif

    // yaw 速度环
    yaw_omega_pid_.SetTarget(GetTargetYawOmega());
    yaw_omega_pid_.SetNow(-imu_yaw_omega_);
    yaw_omega_pid_.CalculatePeriodElapsedCallback();

    // pitch 角度环
#ifdef PITCH_ENCODER_MODE
    pitch_angle_pid_.SetTarget(virtual_pitch_angle_);
    pitch_angle_pid_.SetNow(GetPitchNowAngleNoncumulative());
    pitch_angle_pid_.CalculatePeriodElapsedCallback();
    SetTargetPitchOmega(pitch_angle_pid_.GetOut());
#endif
#ifdef PITCH_IMU_MODE
    pitch_angle_pid_.SetTarget(virtual_pitch_angle_);
    pitch_angle_pid_.SetNow(imu_pitch_angle_);
    pitch_angle_pid_.CalculatePeriodElapsedCallback();
    SetTargetPitchOmega(-pitch_angle_pid_.GetOut());
#endif

    // pitch 速度环
    pitch_omega_pid_.SetTarget(GetTargetPitchOmega());
    pitch_omega_pid_.SetNow(imu_pitch_omega_);
    pitch_omega_pid_.CalculatePeriodElapsedCallback();
}

void Gimbal::SetYawZero()
{
    orb::MotorAdminCmd cmd{};
    cmd.id.bus = orb::MotorBus::CAN3;
    cmd.id.std_id = 0x12;
    cmd.op = orb::MotorAdminOp::SaveZero;
    orb::motor_admin_cmd.publish(cmd);
}

void Gimbal::Output()
{
    // 新方案：直接发布到 orb::motor_cmd
    {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN3;
        c.id.std_id = 0x12;
        c.mode = orb::MotorCtrlMode::Angle;
        c.target_angle = GetTargetYawAngle();
        c.target_omega = GetTargetYawOmega();
        orb::motor_cmd.publish(c);
    }
    {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN3;
        c.id.std_id = 0x11;
        c.mode = orb::MotorCtrlMode::Angle;
        c.target_angle = GetTargetPitchAngle();
        c.target_omega = GetTargetPitchOmega();
        orb::motor_cmd.publish(c);
    }
}

/**
 * @brief 电机就近转位
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // 方案B：业务层不直接访问电机。就近转位基于当前姿态（imu）/目标角，等价处理：将目标 yaw 归一化到与当前 yaw 最近。
    float tmp_delta_angle = fmodf(target_yaw_angle_ - now_yaw_angle_, 2.0f * PI);
    if (tmp_delta_angle > PI) {
        tmp_delta_angle -= 2.0f * PI;
    } else if (tmp_delta_angle < -PI) {
        tmp_delta_angle += 2.0f * PI;
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
    // Robot -> Gimbal 指令（只取最新）
    Subscription<orb::GimbalCmd> gimbal_cmd_sub(orb::gimbal_cmd);

    uint8_t first_run_flag = 0; // 用于标记是否是第一次运行

    // 若本周期没有新指令，则沿用上一帧（避免未初始化使用）
    orb::GimbalCmd cmd{};

    for (;;)
    {
        (void)gimbal_cmd_sub.copy(cmd);

        // 命令：退出（急停/疯车保护）
        if (cmd.request_exit) {
            Exit();
        }

        // 传感器输入
        SetYawImuAngle(cmd.yaw_imu_angle);
        SetYawImuOmega(cmd.yaw_imu_omega);
        SetPitchImuAngle(cmd.pitch_imu_angle);
        SetPitchImuOmega(cmd.pitch_imu_omega);

        // 虚拟目标
        SetVirtualYawAngle(cmd.virtual_yaw_angle);
        SetVirtualPitchAngle(cmd.virtual_pitch_angle);

        // 控制模式/前馈/目标
        SetYawOmegaFeedforword(cmd.yaw_omega_ff);
        SetGimbalYawControlType((cmd.yaw_mode == orb::GimbalYawMode::Omega) ? GIMBAL_CONTROL_TYPE_OMEGA
                                                                            : GIMBAL_CONTROL_TYPE_ANGLE);
        if (cmd.yaw_mode == orb::GimbalYawMode::Omega) {
            SetTargetYawOmega(cmd.target_yaw_omega);
        }

        // 命令：置零/恢复
        if (cmd.request_yaw_zero) {
            SetYawZero();
        }
        if (cmd.request_yaw_recover) {
            orb::MotorAdminCmd admin{};
            admin.id.bus = orb::MotorBus::CAN3;
            admin.id.std_id = 0x12;
            admin.op = orb::MotorAdminOp::ClearError;
            orb::motor_admin_cmd.publish(admin);
            admin.op = orb::MotorAdminOp::Enter;
            orb::motor_admin_cmd.publish(admin);
        }
        if (cmd.request_pitch_recover) {
            orb::MotorAdminCmd admin{};
            admin.id.bus = orb::MotorBus::CAN3;
            admin.id.std_id = 0x11;
            admin.op = orb::MotorAdminOp::ClearError;
            orb::motor_admin_cmd.publish(admin);
            admin.op = orb::MotorAdminOp::Enter;
            orb::motor_admin_cmd.publish(admin);
        }
        if (cmd.request_gimbal_recover) {
            // yaw
            {
                orb::MotorAdminCmd admin{};
                admin.id.bus = orb::MotorBus::CAN3;
                admin.id.std_id = 0x12;
                admin.op = orb::MotorAdminOp::ClearError;
                orb::motor_admin_cmd.publish(admin);
                admin.op = orb::MotorAdminOp::Enter;
                orb::motor_admin_cmd.publish(admin);
            }
            // pitch
            {
                orb::MotorAdminCmd admin{};
                admin.id.bus = orb::MotorBus::CAN3;
                admin.id.std_id = 0x11;
                admin.op = orb::MotorAdminOp::ClearError;
                orb::motor_admin_cmd.publish(admin);
                admin.op = orb::MotorAdminOp::Enter;
                orb::motor_admin_cmd.publish(admin);
            }
        }

        if(first_run_flag == 0){ // 第一次运行到这里，pre_pitch_angle未初始化
            first_run_flag = 1;
            pre_pitch_angle_ = target_pitch_angle_;
        }
        SelfResolution();

        // 发布 gimbal 状态（供 Robot/Chassis 解耦读取）
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