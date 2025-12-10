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
#include "imu.hpp"


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

    void KinematicsInverseResolution();
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

#endif // !APP_CHASSIS_H_