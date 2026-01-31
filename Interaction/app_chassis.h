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

#include "cmsis_os2.h"

class Chassis
{
public:
    void Start();
    void Task();
    void Exit();
    inline void SetTargetVxInGimbal(float target_vx);
    inline void SetTargetVyInGimbal(float target_vy);
    inline void SetTargetVelocityRotation(float target_velocity_rotation);
    inline void SetYawAngle(float yaw_angle);
protected:
    DebugTools  debug_tools_;

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

    bool started_ = false;
    osThreadId_t thread_ = nullptr;

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