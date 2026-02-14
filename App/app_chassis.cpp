/**
 * @file app_chassis.cpp
 * @brief Chassis 实现：订阅输入 Topic，做坐标变换与运动学逆解，发布轮速目标 Topic。
 *
 * 说明：
 * - 本模块不直接控制电机/不直接发送 CAN；只发布 motor-level 的 `orb::dji_c6xx_omega_cmd`。
 * - 电机驱动与 CAN 输出由 Device/actuator 运行时模块负责（CAN TX 仍通过 Topic：`orb::can_tx`）。
 */

#include "app_chassis.h"

#include "math/alg_math.h"

#include "../communication_topic/actuator_cmd_topics.hpp"
#include "../communication_topic/gimbal_state_topics.hpp"
#include "../communication_topic/mcu_topics.hpp"
#include "../Device/motor_ids.hpp"

#include "cmsis_os2.h"
#include "arm_math.h"
extern "C" {
#include "FreeRTOS.h"
#include "task.h" 
}

static_assert(configASSERT_DEFINED == 1, "configASSERT_DEFINED expected");

// 满足 clang-tidy: "Included header FreeRTOS.h is not used directly"
static constexpr uint32_t kFreeRtosTickPeriodMs = portTICK_PERIOD_MS;


Chassis& Chassis::Instance()
{
    static Chassis inst;
    return inst;
}

void Chassis::Init()
{
    if (started_) {
        configASSERT(false);
        return;
    }

    started_ = true;

    static const osThreadAttr_t kChassisTaskAttr = {
        .name = "chassis_task",
        .stack_size = 512,
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
    // 退出时把 4 个轮速清零（按电机 ID 发布）
    static constexpr uint16_t kIds[4] = {
        static_cast<uint16_t>(motor_ids::kWheel1),
        static_cast<uint16_t>(motor_ids::kWheel2),
        static_cast<uint16_t>(motor_ids::kWheel3),
        static_cast<uint16_t>(motor_ids::kWheel4),
    };

    for (uint8_t i = 0; i < 4; ++i) {
        orb::DjiC6xxOmegaCmd c{};
        c.bus = orb::CanBus::CAN1;
        c.rx_std_id = kIds[i];
        c.omega = 0.0f;
        orb::dji_c6xx_omega_cmd.publish(c);
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

    static constexpr uint16_t kIds[4] = {
        static_cast<uint16_t>(motor_ids::kWheel1),
        static_cast<uint16_t>(motor_ids::kWheel2),
        static_cast<uint16_t>(motor_ids::kWheel3),
        static_cast<uint16_t>(motor_ids::kWheel4),
    };

    for (uint8_t i = 0; i < 4; ++i) {
        orb::DjiC6xxOmegaCmd c{};
        c.bus = orb::CanBus::CAN1;
        c.rx_std_id = kIds[i];
        c.omega = w[i];
        orb::dji_c6xx_omega_cmd.publish(c);
    }
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
    static constexpr float kChassisSpeed = 10.0f;
    static constexpr float kChassisSpinSpeed = 30.0f;
    static constexpr float kYawGearRatio = 0.8f;

    Subscription<orb::McuControl> mcu_control_sub(orb::mcu_control);
    Subscription<orb::GimbalState> gimbal_state_sub(orb::gimbal_state);
    orb::McuControl mcu{};
    orb::GimbalState gimbal_state{};

    float spin_speed = 0.0f;
    const float accel = 0.03f;

    for (;;)
    {
        (void)mcu_control_sub.copy(mcu);
        (void)gimbal_state_sub.copy(gimbal_state);

        const float vx_in_gimbal = (mcu.chassis_speed_x - 127.0f) * kChassisSpeed / 128.0f;
        const float vy_in_gimbal = (127.0f - mcu.chassis_speed_y) * kChassisSpeed / 128.0f;
        float v_rotation = (127.0f - mcu.chassis_rotation) * kChassisSpeed / 128.0f;
        const float yaw_angle = -normalize_angle_pm_pi(gimbal_state.yaw_angle / kYawGearRatio);

        bool request_exit = false;
        switch (mcu.chassis_spin)
        {
            case orb::McuChassisSpinMode::Clockwise:
                if (spin_speed < kChassisSpinSpeed)
                    spin_speed = fminf(spin_speed + accel, kChassisSpinSpeed);
                else if (spin_speed > kChassisSpinSpeed)
                    spin_speed = fmaxf(spin_speed - accel, kChassisSpinSpeed);
                v_rotation = spin_speed;
                break;
            case orb::McuChassisSpinMode::Disable:
                spin_speed = 0.0f;
                v_rotation = 0.0f;
                break;
            case orb::McuChassisSpinMode::CounterClockwise:
                request_exit = true;
                break;
            default:
                break;
        }

        SetTargetVxInGimbal(vx_in_gimbal);
        SetTargetVyInGimbal(vy_in_gimbal);
        SetTargetVelocityRotation(v_rotation);
        SetYawAngle(yaw_angle);

        if (request_exit) {
            Exit();
        }

        // 旋转矩阵处理
        RotationMatrixTransform();
        // 运动学逆解算
        KinematicsInverseResolution();

        osDelay(1);// 1khz电机控制频率
    }
}


