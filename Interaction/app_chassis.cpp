// app
#include "FreeRTOS.h"
#include "app_chassis.h"
#include "ins_task.h"
#include "bmi088driver.h"
#include "imu.hpp"
#include "cmsis_os.h"

void Chassis::Init()
{
    // debug_tools_ .VofaInit();
    // 3508电机初始化
    motor_chassis_1_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_2_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_3_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_4_.pid_omega_.Init(1.0f,0.0f,0.0f);

    motor_chassis_1_.Init(&hfdcan1, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_2_.Init(&hfdcan1, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_3_.Init(&hfdcan1, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_OMEGA);
    motor_chassis_4_.Init(&hfdcan1, MOTOR_DJI_ID_0x204, MOTOR_DJI_CONTROL_METHOD_OMEGA);

    motor_chassis_1_.SetTargetOmega(0.0f);
    motor_chassis_2_.SetTargetOmega(0.0f);
    motor_chassis_3_.SetTargetOmega(0.0f);
    motor_chassis_4_.SetTargetOmega(0.0f);

    static const osThreadAttr_t kChassisTaskAttr = {
        .name = "chassis_task",
        .stack_size = 512,//剩余260字节
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Chassis::TaskEntry, this, &kChassisTaskAttr);
}
void Chassis::TaskEntry(void *argument)
{
    Chassis *self = static_cast<Chassis *>(argument);
    self->Task();
}

void Chassis::Exit()
{
    motor_chassis_1_.SetTargetOmega(0.0f);
    motor_chassis_2_.SetTargetOmega(0.0f);
    motor_chassis_3_.SetTargetOmega(0.0f);
    motor_chassis_4_.SetTargetOmega(0.0f);
}
/**
 * @brief 云台系速度 → 底盘系速度 旋转变换
 * @param yaw_angle 云台相对于底盘的偏航角（逆时针为正）
 */
void Chassis::RotationMatrixTransform()
{
    // 旋转矩阵变换
    target_vx_in_chassis_ = cosf(yaw_angle_) * target_vx_in_gimbal_ - sinf(yaw_angle_) * target_vy_in_gimbal_;
    target_vy_in_chassis_ = sinf(yaw_angle_) * target_vx_in_gimbal_ + cosf(yaw_angle_) * target_vy_in_gimbal_;
}

void Chassis::KinematicsInverseResolution()
{
    motor_chassis_1_.SetTargetOmega( (-0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_)
                                    + (target_velocity_rotation_));
    motor_chassis_2_.SetTargetOmega( (-0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_)
                                    + (target_velocity_rotation_));
    motor_chassis_3_.SetTargetOmega( ( 0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_)
                                    + (target_velocity_rotation_));
    motor_chassis_4_.SetTargetOmega( ( 0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_)
                                    + (target_velocity_rotation_));
}

void Chassis::OutputToMotor()
{
    motor_chassis_1_.CalculatePeriodElapsedCallback();
    motor_chassis_2_.CalculatePeriodElapsedCallback();
    motor_chassis_3_.CalculatePeriodElapsedCallback();
    motor_chassis_4_.CalculatePeriodElapsedCallback();

    can_send_data(&hfdcan1, 0x200, g_can1_0x200_tx_data, 8);
}
void Chassis::Task()
{

    for (;;)
    {

        // 旋转矩阵处理
        RotationMatrixTransform();
        // 运动学逆解算
        KinematicsInverseResolution();
        // 输出到底盘电机
        OutputToMotor();
        
        osDelay(pdMS_TO_TICKS(1));// 1khz电机控制频率
    }
}

