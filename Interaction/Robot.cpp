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
#include "app_chassis.h"
#include "alg_math.h"
// module
#include "dvc_MCU_comm.h"
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"

// FreeRTOS stack check
#include "task.h"


void Robot::Init()
{
    dwt_init(480);
    debug_tools_.VofaInit();
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hfdcan2, 0x01, 0x00);
    // 陀螺仪初始化
    // imu_.Init();
    osDelay(pdMS_TO_TICKS(10000));// 10s时间等待陀螺仪收敛

    // 底盘跟随控制PID初始化  17.0f,0.0f,0.0f,5.0f,0.0f,6.0f,0.001f,0.0f,0.0f,0.0f,0.0f
    chassis_follow_pid_.Init(
        17.0f,
        0.0f,
        0.0f,
        5.0f,
        0.0f,
        6.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );
    
    minipc_recive_filter_.Init(40.0f, 0.01f);
    // 云台初始化
    gimbal_.Init();
    // 底盘初始化
    chassis_.Init();
    // ramp_init(&chassis_spin_ramp_source, 0.0005f, 30.0f, -30.0f);
    // 超级电容初始化
    supercap_.Init(&hfdcan3, 0x100, 0x003);

    // 初始化虚拟角度
    // virtual_yaw_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f;
    virtual_yaw_angle_ = 0.0f;
    virtual_pitch_angle_ = 0.0f;

    static const osThreadAttr_t kRobotTaskAttr = {
        .name = "robot_task",
        .stack_size = 768,//剩余492字节
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);
    self->Task();
}

void Robot::Task()
{
    McuCommData mcu_comm_data_local;
    McuCommData mcu_comm_data_local_pre;
    uint8_t first_flag = 0;
    float minipc_recive_yaw_integrate = 0;
    int minipc_division = 0;
    for (;;)
    {
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();
        if(first_flag == 0){
            first_flag = 1;
            mcu_comm_data_local_pre = mcu_comm_data_local;
        }

        /********************** 云台 ***********************/   
        virtual_yaw_angle_ += (127.0f - mcu_comm_data_local.yaw )*YAW_SENSITIVITY_USED_IMU;
        // virtual_pitch_angle_ = (mcu_comm_data_local.pitch_angle - 127.0f)*(PITCH_RANGE_MAX_USE_IMU/128.0f);
        if (virtual_pitch_angle_ >= PITCH_RANGE_MAX_USE_IMU){
            virtual_pitch_angle_ = PITCH_RANGE_MAX_USE_IMU;
        }else if(virtual_pitch_angle_ <= -PITCH_RANGE_MAX_USE_IMU){
            virtual_pitch_angle_ = -PITCH_RANGE_MAX_USE_IMU;
        }

        gimbal_.SetYawImuAngle(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        gimbal_.SetYawImuOmega(mcu_comm_.mcu_imu_data_.yaw_omega_f);
        gimbal_.SetPitchImuAngle(mcu_comm_.mcu_imu_data_.pitch_f);
        gimbal_.SetVirtualYawAngle(virtual_yaw_angle_);
        gimbal_.SetVirtualPitchAngle(virtual_pitch_angle_);

        /********************** 底盘 ***********************/ 
        chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED / 128.0f); //9
        chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED / 128.0f); //9
        chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPEED / 128.0f));
        chassis_.SetYawAngle(-normalize_angle_pm_pi(gimbal_.GetNowYawAngle()/YAW_GEAR_RATIO));
    
        /********************** 模式切换 ***********************/   
        switch(mcu_comm_data_local.chassis_spin)
        {
            case CHASSIS_SPIN_CLOCKWISE:
                chassis_.SetTargetVelocityRotation(CHASSIS_SPIN_SPEED);
                gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_OMEGA);
                gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED);
                gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f)*YAW_SPEED_SENSITIVITY); //补偿速度可能符号错了
            break;
            case CHASSIS_SPIN_DISABLE:
                chassis_.SetTargetVelocityRotation(0.0f);
                gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_ANGLE);
                gimbal_.SetYawOmegaFeedforword(0.0f);
            break;
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE: // 疯车保护
                chassis_.Exit();
                gimbal_.Exit();
            break;
            default:
            // do nothing
            break; 
        };


        /********************** mini PC ***********************/   
        if((fabs(mcu_comm_.mcu_autoaim_data_.pitch_angle) > 0.00f) && (fabs(mcu_comm_.mcu_autoaim_data_.pitch_angle) <0.3f)){
            virtual_pitch_angle_ = - mcu_comm_.mcu_imu_data_.pitch_f
                                   - mcu_comm_.mcu_autoaim_data_.pitch_angle * RAD_TO_DEG;
        }
        // minipc_recive_yaw_integrate += minipc_recive_filter_.GetOutput();
        
        minipc_division ++;
        while (minipc_division == 10){
            virtual_yaw_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f
                               + (mcu_comm_.mcu_autoaim_data_.yaw_angle * RAD_TO_DEG);
            minipc_recive_filter_.Update(virtual_yaw_angle_);
            virtual_yaw_angle_ = minipc_recive_filter_.GetOutput();
            minipc_division = 0;
        }
        
        /********************** 超级电容 ***********************/   
        if(mcu_comm_data_local.supercap == 0){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }else if(mcu_comm_data_local.supercap == 1){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_DISCHARGE);
        }else{
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }
        supercap_.SetPowerLimitMax(100);
        supercap_.SetChargePower(50);
 
        /********************** 调试信息 ***********************/   
        debug_tools_.VofaSendFloat(mcu_comm_.mcu_imu_data_.yaw_total_angle_f); 
        debug_tools_.VofaSendFloat(mcu_comm_.mcu_autoaim_data_.yaw_angle * RAD_TO_DEG);
        debug_tools_.VofaSendFloat(virtual_yaw_angle_);
        // 调试帧尾部
        debug_tools_.VofaSendTail();

        mcu_comm_data_local_pre = mcu_comm_data_local;

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
