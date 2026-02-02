/**
 * @file app_chassis.h
 * @brief 底盘应用层：根据上层输入解算轮速目标，并以 Topic 形式发布给执行器层。
 *
 * **定位**
 * - 这是业务/应用层（Interaction/App），不直接接触 CAN bus/std_id。
 * - 实际电机驱动与 CAN 报文由 Device/MotorActuatorTask 完成。
 *
 * **数据流（以当前实现为准）**
 * - 输入：
 *   - `orb::mcu_control`：遥控/上位机输入（来自 Communication/Device 的解码与 Topic 发布）
 *   - `orb::gimbal_state`：云台姿态（用于云台坐标系到车体坐标系的旋转变换）
 * - 输出：
 *   - `orb::chassis_wheel_omega_cmd`：四个轮子的目标角速度（业务侧输出）
 *
 * **线程模型/守护**
 * - `Start()` 创建 RTOS 任务。
 * - daemon_supervisor 在线判据：收到新的外部输入（mcu_control 或 gimbal_state）才 feed。
 *   这样能避免“任务空转也一直在线”的误判。
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
    static void TaskEntry(void *param);
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

// Module singleton accessor (no Context/service-locator layer)
Chassis& Chassis_Instance();

#endif // !APP_CHASSIS_H_