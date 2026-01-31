// app
#include "app_chassis.h"
#include "cmsis_os2.h"

#include "arm_math.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h" // for taskDISABLE_INTERRUPTS used by configASSERT
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

// 满足 clang-tidy: "Included header FreeRTOS.h is not used directly"
static constexpr uint32_t kFreeRtosTickPeriodMs = portTICK_PERIOD_MS;

#include "../communication_topic/motor_topics.hpp"
#include "../communication_topic/chassis_topics.hpp"

namespace {
}  // namespace

void Chassis::Start()
{
    if (started_) {
        configASSERT(false);
        return;
    }
    started_ = true;

    static const osThreadAttr_t kChassisTaskAttr = {
        .name = "chassis_task",
        .stack_size = 512,//剩余260字节
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Chassis::TaskEntry, this, &kChassisTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}
void Chassis::TaskEntry(void *argument)
{
    Chassis *self = static_cast<Chassis *>(argument);
    self->Task();
}

void Chassis::Exit()
{
    // 退出时把 4 个轮速清零
    for (uint8_t i = 0; i < 4; ++i) {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN1;
        c.id.std_id = static_cast<uint16_t>(0x201 + i);
        c.mode = orb::MotorCtrlMode::Omega;
        c.target_omega = 0.0f;
        orb::motor_cmd.publish(c);
    }
}

void Chassis::KinematicsInverseResolution()
{
    // 业务层只负责计算轮子目标速度
    // 轮序约定：0..3 对应 motor_chassis_1..4
    float w[4];
    w[0] = (-0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_) + (target_velocity_rotation_);
    w[1] = (-0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_) + (target_velocity_rotation_);
    w[2] = ( 0.707107f * target_vx_in_chassis_ - 0.707107f * target_vy_in_chassis_) + (target_velocity_rotation_);
    w[3] = ( 0.707107f * target_vx_in_chassis_ + 0.707107f * target_vy_in_chassis_) + (target_velocity_rotation_);

    for (uint8_t i = 0; i < 4; ++i) {
        orb::MotorCmd c{};
        c.id.bus = orb::MotorBus::CAN1;
        c.id.std_id = static_cast<uint16_t>(0x201 + i);
        c.mode = orb::MotorCtrlMode::Omega;
        c.target_omega = w[i];
        orb::motor_cmd.publish(c);
    }
}

void Chassis::OutputToMotor()
{
    // 统一方案：底盘只发布 orb::motor_cmd。
}
void Chassis::RotationMatrixTransform()
{
    // 将云台坐标系 (vx, vy) 旋转到车体坐标系
    // yaw_angle_ 逆时针为正
    const float c = arm_cos_f32(yaw_angle_);
    const float s = arm_sin_f32(yaw_angle_);

    target_vx_in_chassis_ = c * target_vx_in_gimbal_ - s * target_vy_in_gimbal_;
    target_vy_in_chassis_ = s * target_vx_in_gimbal_ + c * target_vy_in_gimbal_;
}

void Chassis::Task()
{
    // Robot -> Chassis 指令（只取最新）
    Subscription<orb::ChassisCmd> chassis_cmd_sub(orb::chassis_cmd);
    orb::ChassisCmd cmd{};

    for (;;)
    {
        (void)chassis_cmd_sub.copy(cmd);

        // 应用指令
        target_vx_in_gimbal_ = cmd.vx_in_gimbal;
        target_vy_in_gimbal_ = cmd.vy_in_gimbal;
        target_velocity_rotation_ = cmd.v_rotation;
        yaw_angle_ = cmd.yaw_angle;

        if (cmd.request_exit) {
            Exit();
        }

        // 旋转矩阵处理
        RotationMatrixTransform();
        // 运动学逆解算
        KinematicsInverseResolution();
        // 输出到底盘电机
        OutputToMotor();

        osDelay(1);// 1khz电机控制频率
    }
}

