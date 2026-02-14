/**
 * @file app_gimbal.h
 * @brief 云台应用层：角度/角速度控制与自瞄融合，发布云台目标到执行器层。
 *
 * **定位**
 * - 应用层负责控制策略（PID/滤波/插补/模式切换），不直接接触 CAN bus/std_id。
 * - 电机驱动与 CAN 报文由本模块内的电机实例完成（CAN TX 仍通过 Topic：`orb::can_tx`）。
 *
 * **数据流（以当前实现为准）**
 * - 输入：
 *   - `orb::mcu_control`：手动控制/模式开关
 *   - `orb::mcu_autoaim`：miniPC 自瞄输入
 *   - `orb::mcu_imu`：IMU 姿态角与角速度
 * - 输出：
 *   - `orb::dm_mit_cascade_cmd`：云台 DM 串级目标（PID 周期计算在 dm_mit 线程中）
 *   - `orb::dm_mit_admin_cmd`：云台 DM 管理指令（例如 SaveZero/Exit）
 *   - `orb::gimbal_state`：云台姿态状态（供其他模块订阅）
 *
 * **线程模型/守护**
 * - `Start()` 创建 RTOS 任务。
 * - daemon_supervisor 在线判据：收到新的外部输入（mcu_control / mcu_autoaim / mcu_imu）才 feed。
 * - 启动阶段存在 IMU 收敛等待窗口；窗口内仍按“收到新外部数据”喂守护，避免误判。
 */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

// module
#include "filter/low_pass_filter.hpp"

#include "cmsis_os2.h"

// #define YAW_ENCODER_MODE  
#define YAW_IMU_MODE      

// #define PITCH_ENCODER_MODE
#define PITCH_IMU_MODE

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
    static Gimbal& Instance();

    void Init();
    void Task();
    void Exit();
    void SetYawZero();

    inline float GetNowYawAngle();

    inline float GetNowPitchAngle();

    inline float GetNowYawOmega();

    inline float GetNowPitchOmega();

    inline float GetTargetYawOmega();

    inline float GetYawOmegaFeedforword()
    {
        return yaw_omega_feedforword_;
    }

    inline float GetPitchOmegaFeedforword()
    {
        return pitch_omega_feedforword_;
    }

    float GetYawNowAngleNoncumulative()
    {
        return yaw_now_angle_noncumulative_;
    }

    float GetPitchNowAngleNoncumulative()
    {
        return pitch_now_angle_noncumulative_;
    }

    inline void SetTargetYawOmega(float target_yaw_omega);

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

    inline void SetYawImuAngle(float imu_yaw_angle)
    {
        imu_yaw_angle_ = imu_yaw_angle;
    }

    inline void SetYawImuOmega(float imu_yaw_omega)
    {
        imu_yaw_omega_ = imu_yaw_omega;
    }
    
    inline void SetPitchImuAngle(float imu_pitch_angle)
    {
        imu_pitch_angle_ = imu_pitch_angle;
    }

    inline void SetPitchImuOmega(float imu_pitch_omega)
    {
        imu_pitch_omega_ = imu_pitch_omega;
    }

protected:
    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    // pitch轴最小值
    float min_pitch_angle_ = -0.30f;
    // pitch轴最大值
    float max_pitch_angle_ = 0.50f;

    // yaw轴当前角度
    float now_yaw_angle_ = 0.0f;
    // pitch轴当前角度
    float now_pitch_angle_ = 0.0f;

    // yaw轴当前角速度
    float now_yaw_omega_ = 0.0f;
    // pitch轴当前角速度
    float now_pitch_omega_ = 0.0f;

    // 云台状态
    GimbalControlType yaw_control_type_ = GIMBAL_CONTROL_TYPE_ANGLE;

    // yaw轴目标角速度
    float target_yaw_omega_ = 0.0f;

    // yaw轴角速度前馈
    float yaw_omega_feedforword_ = 0.0f;
    // pitch轴角速度前馈
    float pitch_omega_feedforword_ = 0.0f;

    // yaw轴电机非累计角度 rad
    float yaw_now_angle_noncumulative_ = 0.0f;
    // pitch轴电机非累计角度 rad
    float pitch_now_angle_noncumulative_ = 0.0f;

    // yaw轴虚拟轴角度
    float virtual_yaw_angle_ = 0.0f;
    // pitch轴虚拟轴角度
    float virtual_pitch_angle_ = 0.0f;

    // yaw imu角度
    float imu_yaw_angle_ = 0.0f;
    // yaw imu 角速度
    float imu_yaw_omega_ = 0.0f;
    // pitch imu角度
    float imu_pitch_angle_ = 0.0f;
    // pitch imu 角速度
    float imu_pitch_omega_ = 0.0f;

    // miniPC 角度接收滤波（在 Gimbal 模块内完成融合/滤波）
    alg::LowPassFilter minipc_yaw_recive_filter_{};
    alg::LowPassFilter minipc_pitch_recive_filter_{};
    bool minipc_filters_inited_ = false;
    void Output();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
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
 * @brief 获取yaw轴目标角速度
 *
 * @return float yaw轴目标角速度
 */
inline float Gimbal::GetTargetYawOmega()
{
    return (target_yaw_omega_);
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


// Module singleton accessor (no Context/service-locator layer)



#endif // !GIMBAL_H