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
#include "FreeRTOS.h"
#include "app_gimbal.h"
#include "control/alg_pid.h"
#include "cmsis_os2.h"
#include "math/interpolation.hpp"
#include "math/alg_math.h"
#include "filter/low_pass_filter.hpp"
#include "cmsis_os.h"
#include "projdefs.h"
void Gimbal::Init()
{
    // 6220电机初始化
    // motor_yaw_.Init(&hfdcan3, 0x12, 0x01,MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,3.14159f);

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
        alg::DFirst::Disable,
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
        alg::DFirst::Disable,
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
        alg::DFirst::Disable,
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
        alg::DFirst::Disable,
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
        alg::DFirst::Disable,
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
        alg::DFirst::Disable,
        0.01f
    );
    // yaw轴速度环低通滤波器初始化
    yaw_omega_filter_.Init(15.0f,0.001f);
    pitch_omega_filter_.Init(15.0f,0.001f);

    motor_yaw_.Init(&hfdcan3, 0x12, 0x01,MOTOR_DM_CONTROL_METHOD_NORMAL_MIT,12.56637);
    motor_pitch_.Init(&hfdcan3, 0x11, 0x02);

    motor_yaw_.CanSendSaveZero();
    osDelay(pdMS_TO_TICKS(1000));
    motor_yaw_.CanSendClearError();
    motor_pitch_.CanSendClearError();
    osDelay(pdMS_TO_TICKS(1000));
    motor_yaw_.CanSendEnter();
    motor_pitch_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(1000));

    motor_yaw_.SetKp(0); //MIT模式kp
    motor_pitch_.SetKp(0);//26

    motor_yaw_.SetKd(0.0f); // MIT模式kd
    motor_pitch_.SetKd(0.0f);//0.06

    motor_yaw_.SetControlAngle(0);
    motor_pitch_.SetControlAngle(0);

    motor_yaw_.SetControlOmega(0);
    motor_pitch_.SetControlOmega(0);

    motor_yaw_.SetControlTorque(0);
    motor_pitch_.SetControlTorque(0);

    motor_yaw_.Output();
    motor_pitch_.Output();

    // debug_tools_.VofaInit();
    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,//剩余260字节
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
}

void Gimbal::Exit()
{
    motor_yaw_.CanSendExit();
    motor_pitch_.CanSendExit();
}

/**
 * @brief 自身解算
 *
 */
void Gimbal::SelfResolution()
{
    static uint8_t first_resolution_flag = 0;

    now_pitch_angle_ = motor_pitch_.GetNowAngle();
    pitch_now_angle_noncumulative_ = motor_pitch_.GetNowAngleNoncumulative();

    now_yaw_angle_   = motor_yaw_.GetNowAngle();

    // 过零处理
    if(first_resolution_flag == 0){
        first_resolution_flag = 1;
        yaw_zero_angle_ = now_yaw_angle_;
        yaw_relative_zero_angle_ = 0.0f;
    }else{
        yaw_relative_zero_angle_ = get_relative_angle_pm_pi(now_yaw_angle_, yaw_zero_angle_);
    }
    yaw_now_angle_noncumulative_ = motor_yaw_.GetNowAngleNoncumulative();


    now_pitch_omega_ = motor_pitch_.GetNowOmega();
    now_yaw_omega_   = motor_yaw_.GetNowOmega();
    
    now_pitch_torque_ = motor_pitch_.GetNowTorque();
    now_yaw_torque_ = motor_yaw_.GetNowTorque();

    // yaw轴角度环
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
    // yaw轴速度环
    yaw_omega_pid_.SetTarget(GetTargetYawOmega());
    // float filtered_omega = yaw_omega_filter_.Update(GetNowYawOmega());
    yaw_omega_pid_.SetNow(-imu_yaw_omega_);
    yaw_omega_pid_.CalculatePeriodElapsedCallback();

    // pitch轴角度环
#ifdef PITCH_ENCODER_MODE
    // pitch_angle_pid_.SetTarget(virtual_pitch_angle_);
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
    // pitch轴速度环
    pitch_omega_pid_.SetTarget(GetTargetPitchOmega());
    // float pitch_filtered_omega = pitch_omega_filter_.Update(GetNowPitchOmega());
    pitch_omega_pid_.SetNow(imu_pitch_omega_);
    pitch_omega_pid_.CalculatePeriodElapsedCallback();

    // // pitch轴角度归化到±PI / 2之间
    // now_pitch_angle_ = Math_Modulus_Normalization(-motor_pitch_.GetNowAngle(), 2.0f * PI);

}

void Gimbal::SetYawZero()
{
    motor_yaw_.CanSendExit();
    osDelay(pdMS_TO_TICKS(100));
    motor_yaw_.CanSendSaveZero();
    osDelay(pdMS_TO_TICKS(200));
    motor_yaw_.CanSendEnter();
    osDelay(pdMS_TO_TICKS(100));
}
/**
 * @brief 输出到电机
 *
 */
void Gimbal::Output()
{
    // motor_yaw_.SetControlOmega(target_yaw_omega_ + yaw_omega_feedforword_);
    motor_yaw_.SetControlTorque(yaw_omega_pid_.GetOut());
    motor_pitch_.SetControlTorque(pitch_omega_pid_.GetOut());
    motor_yaw_.SendPeriodElapsedCallback();
    motor_pitch_.SendPeriodElapsedCallback();
}

/**
 * @brief 电机就近转位
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // Yaw就近转位
    float tmp_delta_angle;
    tmp_delta_angle = fmod(target_yaw_angle_ - now_yaw_angle_, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    target_yaw_angle_ = motor_yaw_.GetNowAngle() + tmp_delta_angle;

    // // Pitch就近转位
    // Math_Constrain(&target_pitch_angle_, Min_Pitch_Angle, Max_Pitch_Angle);
    // tmp_delta_angle = target_pitch_angle_ - now_pitch_angle_;
    // target_pitch_angle_ = -motor_pitch_.GetNowAngle() + tmp_delta_angle;
}

void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);
    self->Task();  
}

void Gimbal::Task()
{
    uint32_t loop_count = 0; // 循环计数，用于奇偶次分配 Alive 回调
  
    for (;;)
    {
        if(loop_count == 0){ // 第一次运行到这里，pre_pitch_angle未初始化
            pre_pitch_angle_ = target_pitch_angle_;
        }
        SelfResolution();
        if (loop_count & 1) {
            motor_yaw_.AlivePeriodElapsedCallback();
        } else {
            motor_pitch_.AlivePeriodElapsedCallback();
        }
        Output();
        pre_pitch_angle_ = target_pitch_angle_;
        loop_count++;
        osDelay(pdMS_TO_TICKS(1)); // 1khz电机控制频率
    }
}