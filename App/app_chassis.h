/**
 * @file app_chassis.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_


// module
#include "debug_tools.h"
#include "dvc_motor_dji.h"


class Chassis
{
public:
    // 底盘4个3508， 控制全向轮
    MotorDjiC620 motor_chassis_1_,
                 motor_chassis_2_,
                 motor_chassis_3_,
                 motor_chassis_4_;

    DebugTools  debug_tools_;
    void Init();
    void Task();
    void Exit();
    inline void SetTargetVxInGimbal(float target_vx);
    inline void SetTargetVyInGimbal(float target_vy);
    inline void SetTargetVelocityRotation(float target_velocity_rotation);
    inline void SetYawAngle(float yaw_angle);
    inline void SetPowerLimit(float power_limit_w);
    inline void SetPowerBufferEnergy(float power_buffer_energy);
    inline float GetPowerEstimate();
    inline float GetPowerScale();
    inline float GetPowerBufferConsume();
protected:
    // 云台坐标系目标速度
    float target_vx_in_gimbal_ = 0.0f;
    float target_vy_in_gimbal_ = 0.0f;
    // 底盘坐标系目标速度
    float target_vx_in_chassis_ = 0.0f;
    float target_vy_in_chassis_ = 0.0f;
    // 目标速度 旋转
    float target_velocity_rotation_ = 0.0f;
    // 云台相对于底盘的偏航角（逆时针为正）
    float yaw_angle_ = 0.0f; 

    // 底盘功率限流反馈（来自裁判系统缓冲能量）
    float chassis_power_estimate_ = 0.0f;
    float chassis_power_limit_w_ = 60.0f;
    float power_buffer_energy_ = 60.0f;
    float power_buffer_consume_ = 0.0f;
    float power_buffer_consume_last_ = 0.0f;
    float power_scale_ = 1.0f;
    float power_pd_kp_ = 1.15f;
    float power_pd_kd_ = 0.50f;
    float power_scale_min_ = 0.02f;
    float power_scale_attack_alpha_ = 0.35f;
    float power_scale_release_alpha_ = 0.015f;
    float power_hard_limit_trigger_j_ = 1.0f;
    float power_hard_limit_scale_ = 0.02f;
    uint16_t power_hard_limit_hold_ticks_ = 0;
    uint16_t power_hard_limit_hold_ticks_default_ = 300;

    void KinematicsInverseResolution();
    void ChassisPidCalculate();
    void ChassisPowerControl();
    void RotationMatrixTransform();
    void OutputToMotor();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/**
 * @brief 设定目标速度X
 *
 * @param target_velocity_x 目标速度X
 */
inline void Chassis::SetTargetVxInGimbal(float target_vx)
{
    target_vx_in_gimbal_ = target_vx;
}

/**
 * @brief 设定目标速度Y
 *
 * @param target_velocity_y 目标速度Y
 */
inline void Chassis::SetTargetVyInGimbal(float target_vy)
{
    target_vy_in_gimbal_ = target_vy;
}

/**
 * @brief 设定目标速度旋转
 *
 * @param target_velocity_rotation 目标速度Y
 */
inline void Chassis::SetTargetVelocityRotation(float target_velocity_rotation)
{
    target_velocity_rotation_ = target_velocity_rotation;
}

inline void Chassis::SetYawAngle(float yaw_angle)
{
    yaw_angle_ = yaw_angle;
}

inline void Chassis::SetPowerLimit(float power_limit_w)
{
    if (power_limit_w > 1.0f)
    {
        chassis_power_limit_w_ = power_limit_w;
    }
}

inline void Chassis::SetPowerBufferEnergy(float power_buffer_energy)
{
    if (power_buffer_energy < 0.0f)
    {
        power_buffer_energy_ = 0.0f;
    }
    else if (power_buffer_energy > 60.0f)
    {
        power_buffer_energy_ = 60.0f;
    }
    else
    {
        power_buffer_energy_ = power_buffer_energy;
    }
}

inline float Chassis::GetPowerEstimate()
{
    return chassis_power_estimate_;
}

inline float Chassis::GetPowerScale()
{
    return power_scale_;
}

inline float Chassis::GetPowerBufferConsume()
{
    return power_buffer_consume_;
}

#endif // !APP_CHASSIS_H_