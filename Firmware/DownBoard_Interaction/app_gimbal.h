/**
 * @file app_gimbal.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

// module
#include "debug_tools.h"
#include "dvc_motor_dm.h"
#include "interpolation.hpp"
#include "low_pass_filter.hpp"


/**
 * @brief 云台控制类型
 *
 */
enum GimbalControlType
{
    GIMBAL_CONTROL_TYPE_MANUAL = 0,
    GIMBAL_CONTROL_TYPE_AUTOAIM,
    GIMBAL_CONTROL_TYPE_OMEGA, // 角速度控制模式
    GIMBAL_CONTROL_TYPE_ANGLE, // 角度控制模式
};



class Gimbal
{
public:
    // 2个DM6220，作为云台Yaw和Pitch轴控制电机
    MotorDmNormal motor_yaw_;
    MotorDmNormal motor_pitch_;

    Interpolation pitch_angle_interpolation; // pitch角插补类实例

    // 云台yaw轴角度环pid
    Pid yaw_angle_pid_;
    // 云台pitch轴角度环pid
    Pid pitch_angle_pid_;
    // 云台yaw轴速度环pid
    Pid yaw_omega_pid_;
    // 云台pitch轴速度环pid
    Pid pitch_omega_pid_;

    LowPassFilter yaw_omega_filter_;
    LowPassFilter pitch_omega_filter_;

    DebugTools debug_tools_;
    
    void Init();
    void Task();
    void Exit();

    inline float GetNowYawAngle();

    inline float GetNowPitchAngle();

    inline float GetNowYawOmega();

    inline float GetNowPitchOmega();

    inline float GetNowYawTorque()
    {
        return now_yaw_torque_;
    }

    inline float GetNowPitchTorque()
    {
        return now_pitch_torque_;
    }
    inline float GetTargetYawAngle();

    inline float GetTargetPitchAngle();

    inline float GetTargetYawOmega();

    inline float GetTargetPitchOmega();

    inline float GetYawOmegaFeedforword()
    {
        return yaw_omega_feedforword_;
    }

    inline float GetPitchOmegaFeedforword()
    {
        return pitch_omega_feedforword_;
    }

    float GetYawRelativeZeroAngle()
    {
        return yaw_relative_zero_angle_;
    }

    float GetYawNowAngleNoncumulative()
    {
        return yaw_now_angle_noncumulative_;
    }

    float GetPitchNowAngleNoncumulative()
    {
        return pitch_now_angle_noncumulative_;
    }

    inline void SetTargetYawAngle(float target_yaw_angle);

    inline void SetTargetPitchAngle(float target_pitch_angle);

    inline void SetTargetYawOmega(float target_yaw_omega);

    inline void SetTargetPitchOmega(float target_pitch_omega);

    inline void SetTargetYawTorque(float torque)
    {
        target_yaw_torque_ = torque;
    }

    inline void SetTargetPitchTorque(float torque)
    {
        target_pitch_torque_ = torque;
    }

    inline void SetYawOmegaFeedforword(float yaw_omega_feedforword)
    {
        yaw_omega_feedforword_ = yaw_omega_feedforword;
    }

    inline void SetPitchOmegaFeedforword(float pitch_omega_feedforword)
    {
        pitch_omega_feedforword_ = pitch_omega_feedforword;
    }

    inline void SetVirtualYawAngle(float virtual_yaw_angle)
    {
        virtual_yaw_angle_ = virtual_yaw_angle;
    }

    inline void SetVirtualPitchAngle(float virtual_pitch_angle)
    {
        virtual_pitch_angle_ = virtual_pitch_angle;
    }
    
    inline void SetGimbalYawControlType(GimbalControlType gimbal_control_type)
    {
        yaw_control_type_ = gimbal_control_type;
    }

    inline void SetGimbalPitchControlType(GimbalControlType gimbal_control_type)
    {
        pitch_control_type_ = gimbal_control_type;
    }


protected:
    // pitch轴最小值
    float min_pitch_angle_ = -0.30f;
    // pitch轴最大值
    float max_pitch_angle_ = 0.50f;

    // 用于pitch角的插补算法
    float pre_pitch_angle_ = 0.0f; 

    // yaw轴当前角度
    float now_yaw_angle_ = 0.0f;
    // pitch轴当前角度
    float now_pitch_angle_ = 0.0f;

    // yaw轴当前角速度
    float now_yaw_omega_ = 0.0f;
    // pitch轴当前角速度
    float now_pitch_omega_ = 0.0f;

    // yaw轴当前力矩
    float now_yaw_torque_ = 0.0f;
    // pitch轴当前力矩
    float now_pitch_torque_ = 0.0f;

    // 云台状态
    GimbalControlType gimbal_control_type_ = GIMBAL_CONTROL_TYPE_MANUAL;
    GimbalControlType yaw_control_type_ = GIMBAL_CONTROL_TYPE_ANGLE;
    GimbalControlType pitch_control_type_ = GIMBAL_CONTROL_TYPE_ANGLE;
    
    // yaw轴目标角度
    float target_yaw_angle_ = 0.0f;
    // pitch轴目标角度
    float target_pitch_angle_ = 0.0f;

    // yaw轴目标角速度
    float target_yaw_omega_ = 0.0f;
    // pitch轴目标角速度
    float target_pitch_omega_ = 0.0f;

    // yaw轴目标力矩
    float target_yaw_torque_ = 0.0f;
    //pitch轴目标力矩
    float target_pitch_torque_ = 0.0f;

    // yaw轴角速度前馈
    float yaw_omega_feedforword_ = 0.0f;
    // pitch轴角速度前馈
    float pitch_omega_feedforword_ = 0.0f;

    // yaw轴相对于上电时的角度
    float yaw_relative_zero_angle_ = 0.0f;

    // yaw轴上电时的绝对角度
    float yaw_zero_angle_ = 0.0f;

    // yaw轴电机非累计角度 rad
    float yaw_now_angle_noncumulative_ = 0.0f;
    // pitch轴电机非累计角度 rad
    float pitch_now_angle_noncumulative_ = 0.0f;

    // yaw轴虚拟轴角度
    float virtual_yaw_angle_ = 0.0f;
    // pitch轴虚拟轴角度
    float virtual_pitch_angle_ = 0.0f;

    void SelfResolution();
    void MotorNearestTransposition();
    void Output();
    static void TaskEntry(void *param);
};

/**
 * @brief 获取yaw轴当前角度
 *
 * @return float yaw轴当前角度
 */
inline float Gimbal::GetNowYawAngle()
{
    return (now_yaw_angle_);
}

/**
 * @brief 获取pitch轴当前角度
 *
 * @return float pitch轴当前角度
 */
inline float Gimbal::GetNowPitchAngle()
{
    return (now_pitch_angle_);
}

/**
 * @brief 获取yaw轴当前角速度
 *
 * @return float yaw轴当前角速度
 */
inline float Gimbal::GetNowYawOmega()
{
    return (now_yaw_omega_);
}

/**
 * @brief 获取pitch轴当前角速度
 *
 * @return float pitch轴当前角速度
 */
inline float Gimbal::GetNowPitchOmega()
{
    return (now_pitch_omega_);
}

/**
 * @brief 获取yaw轴目标角度
 *
 * @return float yaw轴目标角度
 */
inline float Gimbal::GetTargetYawAngle()
{
    return (target_yaw_angle_);
}

/**
 * @brief 获取pitch轴目标角度
 *
 * @return float pitch轴目标角度
 */
inline float Gimbal::GetTargetPitchAngle()
{
    return (target_pitch_angle_);
}

/**
 * @brief 获取yaw轴目标角速度
 *
 * @return float yaw轴目标角速度
 */
inline float Gimbal::GetTargetYawOmega()
{
    return (target_yaw_omega_);
}

/**
 * @brief 获取pitch轴目标角速度
 *
 * @return float pitch轴目标角速度
 */
inline float Gimbal::GetTargetPitchOmega()
{
    return (target_pitch_omega_);
}

/**
 * @brief 设定yaw轴角度
 *
 * @param target_yaw_angle yaw轴角度
 */
inline void Gimbal::SetTargetYawAngle(float target_yaw_angle)
{
    target_yaw_angle_ = target_yaw_angle;
}

/**
 * @brief 设定pitch轴角度
 *
 * @param target_pitch_angle pitch轴角度
 */
inline void Gimbal::SetTargetPitchAngle(float target_pitch_angle)
{
    target_pitch_angle_ = target_pitch_angle;
    pitch_angle_interpolation.Start(pre_pitch_angle_, target_pitch_angle_, 8);
}

/**
 * @brief 设定yaw轴角速度
 *
 * @param target_yaw_omega yaw轴角速度
 */
inline void Gimbal::SetTargetYawOmega(float target_yaw_omega)
{
    target_yaw_omega_ = target_yaw_omega;
}

/**
 * @brief 设定pitch轴角速度
 *
 * @param target_pitch_omega pitch轴角速度
 */
inline void Gimbal::SetTargetPitchOmega(float target_pitch_omega)
{
    target_pitch_omega_ = target_pitch_omega;
}


#endif // !GIMBAL_H