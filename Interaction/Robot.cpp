/**
 * @file Robot.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Robot.h"
#include "cmsis_os2.h"

// app
#include "alg_math.h"

// Topic pub-sub
#include "../communication_topic/topic_pubsub.hpp"
#include "../communication_topic/mcu_topics.hpp"
#include "../communication_topic/device_topics.hpp"
#include "../communication_topic/gimbal_topics.hpp"
#include "../communication_topic/gimbal_state_topics.hpp"
#include "../communication_topic/chassis_topics.hpp"
#include "../communication_topic/topic_wait.hpp"


void Robot::Start()
{
    if (started_) {
        configASSERT(false);
        return;
    }
    started_ = true;

    // 只负责启动任务：不做任何外设 bring-up，也尽量不做耗时延迟。
    static const osThreadAttr_t kRobotTaskAttr = {
        .name = "robot_task",
        .stack_size = 768,//剩余492字节
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
    if (!thread_) {
        configASSERT(false);
    }
}

void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);
    self->Task();
}

void Robot::Task()
{
    // 任务内初始化：保证对象构造后即可“逻辑可用”，无需显式 Init。
    osDelay(3000); // 3s 等待陀螺仪收敛

    minipc_yaw_recive_filter_.Init(20.0f, 0.001f);
    minipc_pitch_recive_filter_.Init(20.0f, 0.001f);

    // 初始化虚拟角度
    virtual_yaw_angle_ = 0.0f;
    virtual_pitch_angle_ = 0.0f;

    // 通过 Topic 获取最新数据（避免直接读 mcu_comm_ 内部 volatile 并关中断）
    Sub<orb::McuControl> mcu_control_sub(orb::mcu_control);
    Sub<orb::McuAutoAim> mcu_autoaim_sub(orb::mcu_autoaim);
    Sub<orb::McuImu> mcu_imu_sub(orb::mcu_imu);

    // 事件驱动：只有“输入 topic”唤醒 Robot（Robot 自己的输出不应作为唤醒源）
    TopicWaiter wake_waiter(1u << 0);
    orb::mcu_control.register_notifier(wake_waiter.notifier());
    orb::mcu_autoaim.register_notifier(wake_waiter.notifier());
    orb::mcu_imu.register_notifier(wake_waiter.notifier());

    orb::McuControl mcu_comm_data_local{};
    orb::McuControl mcu_comm_data_local_pre{};
    orb::McuAutoAim autoaim_local{};
    orb::McuImu imu_local{};

    uint8_t first_flag = 0;

    float spin_speed = 0.0f;
    float accel = 0.03f;

    // gimbal 指令缓存（无更新时沿用上一帧）
    orb::GimbalCmd gimbal_cmd{};

    // chassis 指令缓存（无更新时沿用上一帧）
    orb::ChassisCmd chassis_cmd{};

    // gimbal 状态订阅（用于给 chassis 提供 yaw 角度）
    Subscription<orb::GimbalState> gimbal_state_sub(orb::gimbal_state);
    orb::GimbalState gimbal_state{};

    for (;;)
    {
        // 阻塞等待事件；同时设一个上限超时，避免在完全无事件时控制链路停转
        wake_waiter.wait(1);

        // 拉取最新数据（若无更新则沿用上一帧）
        (void)mcu_control_sub.copy(mcu_comm_data_local);
        (void)mcu_autoaim_sub.copy(autoaim_local);
        (void)mcu_imu_sub.copy(imu_local);
        (void)gimbal_state_sub.copy(gimbal_state);

        if(first_flag == 0){
            first_flag = 1;
            mcu_comm_data_local_pre = mcu_comm_data_local;
        }

        /********************** mini PC ***********************/
        if(mcu_comm_data_local.auto_aim_flag == 1) {
            virtual_pitch_angle_ =  imu_local.pitch_f
                                    - autoaim_local.pitch_angle * RAD_TO_DEG;
            minipc_pitch_recive_filter_.Update(virtual_pitch_angle_);
            virtual_pitch_angle_ = minipc_pitch_recive_filter_.GetOutput();

            virtual_yaw_angle_ = imu_local.yaw_total_angle_f
                                    + (autoaim_local.yaw_angle * RAD_TO_DEG);
            minipc_yaw_recive_filter_.Update(virtual_yaw_angle_);
            virtual_yaw_angle_ = minipc_yaw_recive_filter_.GetOutput();
        }

        /********************** 云台（融合输出） ***********************/
        virtual_yaw_angle_ += (127.0f - mcu_comm_data_local.yaw )*YAW_SENSITIVITY_USED_IMU;
        if(mcu_comm_data_local.auto_aim_flag == 0) {
            virtual_pitch_angle_ = (127.0f - mcu_comm_data_local.pitch_angle )*(2 * PITCH_RANGE_MAX_USE_IMU/128.0f);
        }
        if (virtual_pitch_angle_ >= 1.5 * PITCH_RANGE_MAX_USE_IMU){
            virtual_pitch_angle_ = 1.5 * PITCH_RANGE_MAX_USE_IMU;
        }else if(virtual_pitch_angle_ <= (-3 * PITCH_RANGE_MAX_USE_IMU)){
            virtual_pitch_angle_ = (-3 * PITCH_RANGE_MAX_USE_IMU);
        }

        // 组装并发布 Robot->Gimbal 指令（Topic 化解耦）
        gimbal_cmd.yaw_imu_angle = imu_local.yaw_total_angle_f;
        gimbal_cmd.yaw_imu_omega = imu_local.yaw_omega_f;
        gimbal_cmd.pitch_imu_angle = imu_local.pitch_f;
        gimbal_cmd.pitch_imu_omega = imu_local.pitch_omega_f;
        gimbal_cmd.virtual_yaw_angle = virtual_yaw_angle_;
        gimbal_cmd.virtual_pitch_angle = virtual_pitch_angle_;

        // 默认：角度环
        gimbal_cmd.yaw_mode = orb::GimbalYawMode::Angle;
        gimbal_cmd.yaw_omega_ff = 0.0f;
        gimbal_cmd.target_yaw_omega = 0.0f;
        gimbal_cmd.request_yaw_zero = false;
        gimbal_cmd.request_yaw_recover = false;
        gimbal_cmd.request_pitch_recover = false;
        gimbal_cmd.request_gimbal_recover = false;
        gimbal_cmd.request_exit = false;

        /********************** 底盘（融合输出） ***********************/
        chassis_cmd.vx_in_gimbal = (mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED / 128.0f;
        chassis_cmd.vy_in_gimbal = (127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED / 128.0f;
        chassis_cmd.v_rotation = ((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPEED / 128.0f);
        chassis_cmd.yaw_angle = -normalize_angle_pm_pi(gimbal_state.yaw_angle / YAW_GEAR_RATIO);
        chassis_cmd.request_exit = false;

        /********************** 模式切换 ***********************/
        switch(mcu_comm_data_local.chassis_spin)
        {
            case orb::McuChassisSpinMode::Clockwise:
                if (spin_speed < CHASSIS_SPIN_SPEED)
                    spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED);
                else if (spin_speed > CHASSIS_SPIN_SPEED)
                    spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED);

                chassis_cmd.v_rotation = spin_speed;

                // Robot->Gimbal: yaw 速度模式
                gimbal_cmd.yaw_mode = orb::GimbalYawMode::Omega;
                gimbal_cmd.yaw_omega_ff = (YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED);
                gimbal_cmd.target_yaw_omega = (mcu_comm_data_local.yaw - 127.0f)*YAW_SPEED_SENSITIVITY;
            break;
            case orb::McuChassisSpinMode::Disable:
                spin_speed = 0;
                chassis_cmd.v_rotation = 0.0f;
            break;
            case orb::McuChassisSpinMode::CounterClockwise: // 疯车保护
                chassis_cmd.request_exit = true;
                gimbal_cmd.request_exit = true;
            break;
            default:
            break;
        };

        // reset_zero：保持阻塞语义由 gimbal 内部实现，这里仅发布命令
        if (mcu_comm_data_local.reset_zero == 1)
        {
            gimbal_cmd.request_yaw_zero = true;
            gimbal_cmd.request_gimbal_recover = true;
        }

        // 发布（publish 即唤醒对应任务）
        orb::gimbal_cmd.publish(gimbal_cmd);
        orb::chassis_cmd.publish(chassis_cmd);

        /********************** 超级电容（融合输出） ***********************/
        {
            orb::SupercapTx tx{};
            tx.supercap_enable_status = 1;

            switch (mcu_comm_data_local.supercap) {
                case orb::SupercapUserCmd::Charge:
                    tx.supercap_charge_status = orb::SupercapChargeMode::Charge;
                    break;
                case orb::SupercapUserCmd::Discharge:
                    tx.supercap_charge_status = orb::SupercapChargeMode::Discharge;
                    break;
                default:
                    tx.supercap_charge_status = orb::SupercapChargeMode::Charge;
                    break;
            }

            tx.power_limit_max = 100;
            tx.charge_power = 50;

            orb::supercap_tx.publish(tx);
        }

        mcu_comm_data_local_pre = mcu_comm_data_local;
    }
}
