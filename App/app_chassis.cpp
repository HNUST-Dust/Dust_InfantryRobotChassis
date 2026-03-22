// app
#include "FreeRTOS.h"
#include "app_chassis.h"
#include "cmsis_os.h"

void Chassis::Init()
{
    // debug_tools_ .VofaInit();
    // 3508电机初始化
    motor_chassis_1_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_2_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_3_.pid_omega_.Init(1.0f,0.0f,0.0f);
    motor_chassis_4_.pid_omega_.Init(1.0f,0.0f,0.0f);

    motor_chassis_1_.Init(&hfdcan1, MOTOR_DJI_ID_0x201, MOTOR_DJI_CONTROL_METHOD_CURRENT);
    motor_chassis_2_.Init(&hfdcan1, MOTOR_DJI_ID_0x202, MOTOR_DJI_CONTROL_METHOD_CURRENT);
    motor_chassis_3_.Init(&hfdcan1, MOTOR_DJI_ID_0x203, MOTOR_DJI_CONTROL_METHOD_CURRENT);
    motor_chassis_4_.Init(&hfdcan1, MOTOR_DJI_ID_0x204, MOTOR_DJI_CONTROL_METHOD_CURRENT);

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

void Chassis::ChassisPidCalculate()
{
    motor_chassis_1_.SetTargetCurrent(motor_chassis_1_.pid_omega_.update(
        motor_chassis_1_.GetTargetOmega(), motor_chassis_1_.GetNowOmega()));
    motor_chassis_2_.SetTargetCurrent(motor_chassis_2_.pid_omega_.update(
        motor_chassis_2_.GetTargetOmega(), motor_chassis_2_.GetNowOmega()));
    motor_chassis_3_.SetTargetCurrent(motor_chassis_3_.pid_omega_.update(
        motor_chassis_3_.GetTargetOmega(), motor_chassis_3_.GetNowOmega()));
    motor_chassis_4_.SetTargetCurrent(motor_chassis_4_.pid_omega_.update(
        motor_chassis_4_.GetTargetOmega(), motor_chassis_4_.GetNowOmega()));
}

void Chassis::ChassisPowerControl()
{
    power_buffer_consume_ = 60.0f - power_buffer_energy_;
    if (power_buffer_consume_ < 0.0f)
    {
        power_buffer_consume_ = 0.0f;
    }
    else if (power_buffer_consume_ > 60.0f)
    {
        power_buffer_consume_ = 60.0f;
    }
    chassis_power_estimate_ = power_buffer_consume_;

    const float consume_ratio = power_buffer_consume_ / 60.0f;
    const float consume_delta = power_buffer_consume_ - power_buffer_consume_last_;
    power_buffer_consume_last_ = power_buffer_consume_;

    if (power_buffer_energy_ < power_hard_limit_trigger_j_)
    {
        power_hard_limit_hold_ticks_ = power_hard_limit_hold_ticks_default_;
    }

    if (power_hard_limit_hold_ticks_ > 0)
    {
        power_hard_limit_hold_ticks_--;
        power_scale_ = power_hard_limit_scale_;

        motor_chassis_1_.SetTargetCurrent(motor_chassis_1_.GetTargetCurrent() * power_scale_);
        motor_chassis_2_.SetTargetCurrent(motor_chassis_2_.GetTargetCurrent() * power_scale_);
        motor_chassis_3_.SetTargetCurrent(motor_chassis_3_.GetTargetCurrent() * power_scale_);
        motor_chassis_4_.SetTargetCurrent(motor_chassis_4_.GetTargetCurrent() * power_scale_);
        return;
    }

    // 消耗越大，目标缩放越小；采用二次项提升高消耗区抑制强度。
    float target_scale = 1.0f - power_pd_kp_ * consume_ratio * consume_ratio;
    if (consume_delta > 0.0f)
    {
        target_scale -= power_pd_kd_ * (consume_delta / 60.0f);
    }

    // 低缓冲区间硬限幅：缓冲越低，允许的最大电流比例越小。
    if (power_buffer_energy_ < 2.0f)
    {
        if (target_scale > 0.05f)
        {
            target_scale = 0.05f;
        }
    }
    else if (power_buffer_energy_ < 5.0f)
    {
        if (target_scale > 0.10f)
        {
            target_scale = 0.10f;
        }
    }
    else if (power_buffer_energy_ < 10.0f)
    {
        if (target_scale > 0.18f)
        {
            target_scale = 0.18f;
        }
    }
    else if (power_buffer_energy_ < 20.0f)
    {
        if (target_scale > 0.30f)
        {
            target_scale = 0.30f;
        }
    }

    if (target_scale > 1.0f)
    {
        target_scale = 1.0f;
    }
    else if (target_scale < power_scale_min_)
    {
        target_scale = power_scale_min_;
    }

    const float alpha = (target_scale < power_scale_) ? power_scale_attack_alpha_ : power_scale_release_alpha_;
    power_scale_ += alpha * (target_scale - power_scale_);

    motor_chassis_1_.SetTargetCurrent(motor_chassis_1_.GetTargetCurrent() * power_scale_);
    motor_chassis_2_.SetTargetCurrent(motor_chassis_2_.GetTargetCurrent() * power_scale_);
    motor_chassis_3_.SetTargetCurrent(motor_chassis_3_.GetTargetCurrent() * power_scale_);
    motor_chassis_4_.SetTargetCurrent(motor_chassis_4_.GetTargetCurrent() * power_scale_);
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
        // 底盘任务中进行电机速度环PID计算
        ChassisPidCalculate();
        // 基于电机功率预测的总功率PD限流
        ChassisPowerControl();
        // 输出到底盘电机
        OutputToMotor();
        
        osDelay(pdMS_TO_TICKS(1));// 1khz电机控制频率
    }
}

