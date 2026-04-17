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

// app
#include "app_chassis.h"
#include "dvc_referee.h"
#include "math/alg_math.h"
#include "bmi088.h"
// module
#include "dvc_MCU_comm.h"
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"

// FreeRTOS stack check
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <cmath>

// #define VAR_SPIN_MODE
namespace {
float AlignSingleTurnYawToNearestMultiTurnRad(float imu_multi_turn_rad, float single_turn_yaw_rad)
{
    const float kTwoPi = 2.0f * PI;
    const float turns = roundf((imu_multi_turn_rad - single_turn_yaw_rad) / kTwoPi);
    return single_turn_yaw_rad + (turns * kTwoPi);
}
}


void Robot::Init()
{
    dwt_init(480);
    debug_tools_.VofaInit();
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hfdcan2, 0x01, 0x00);
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
    
    minipc_yaw_recive_filter_.Init(20.0f, 0.001f);
    minipc_pitch_recive_filter_.Init(20.0f, 0.001f);
    // 裁判系统初始化
    referee_.Init();

    // 底盘初始化
    chassis_.Init();
    // ramp_init(&chassis_spin_ramp_source, 0.0005f, 30.0f, -30.0f);
    // 超级电容初始化
    // supercap_.Init(&hfdcan3, 0x100, 0x003);
    // 云台初始化
    gimbal_.Init();
    // 板载陀螺仪初始化
    bmi088_.Start();
    
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

    float yaw_avg_buffer[10] = {0};
    uint8_t yaw_avg_index = 0;
    uint8_t yaw_avg_count = 0;
    float yaw_avg_sum = 0.0f;
    float spin_speed = 0.0f;
    float accel = 0.03f;
    // 变速小陀螺（用于 test_spin）
    float var_spin_phase = 0.0f;
    const float var_spin_freq = 0.016f;
    const float var_spin_min_ratio = 0.8f;
    const float var_spin_max_ratio = 1.2f;

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
        if(mcu_comm_data_local.Switch.auto_aim_flag == 0) {
            virtual_pitch_angle_ = (mcu_comm_data_local.pitch_angle - 127.0f )*(PITCH_RANGE_MAX_USE_IMU/128.0f);
        }
        if (virtual_pitch_angle_ >= PITCH_RANGE_MAX_USE_IMU){
            virtual_pitch_angle_ = PITCH_RANGE_MAX_USE_IMU;
        }else if(virtual_pitch_angle_ <= (-0.4f * PITCH_RANGE_MAX_USE_IMU)){
            virtual_pitch_angle_ = (-0.4f * PITCH_RANGE_MAX_USE_IMU);
        }

        const bool gimbal_power_on = referee_.IsGimbalPowerOn();
        if (!last_gimbal_power_on_ && gimbal_power_on) {
            // 在 Robot 侧处理上电沿对齐：将 BMI088 单圈 yaw 映射到 IMU 多圈角最近等效角。
            virtual_yaw_angle_ = AlignSingleTurnYawToNearestMultiTurnRad(
                mcu_comm_.mcu_imu_data_.yaw_total_angle_f,
                bmi088_.yaw_rad
            );
            gimbal_wait_align_then_reset_zero_ = true;
            gimbal_align_stable_ticks_ = 0;
        } else if (!gimbal_power_on) {
            gimbal_wait_align_then_reset_zero_ = false;
            gimbal_align_stable_ticks_ = 0;
        }
        last_gimbal_power_on_ = gimbal_power_on;

        gimbal_.SetYawImuAngle(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        gimbal_.SetYawImuOmega(mcu_comm_.mcu_imu_data_.yaw_omega_f);
        gimbal_.SetPitchImuAngle(mcu_comm_.mcu_imu_data_.pitch_f);
        gimbal_.SetPitchImuOmega(mcu_comm_.mcu_imu_data_.pitch_omega_f);
        gimbal_.SetVirtualYawAngle(virtual_yaw_angle_);
        gimbal_.SetVirtualPitchAngle(virtual_pitch_angle_);

        gimbal_.SetPowerOnFlag(gimbal_power_on);

        if (gimbal_wait_align_then_reset_zero_ && gimbal_power_on) {
            static constexpr float kAlignDoneThresholdRad = 0.05f;
            static constexpr uint16_t kAlignStableTicksRequired = 20;

            const float yaw_err = get_relative_angle_pm_pi(
                mcu_comm_.mcu_imu_data_.yaw_total_angle_f,
                virtual_yaw_angle_
            );

            if (fabsf(yaw_err) <= kAlignDoneThresholdRad) {
                if (++gimbal_align_stable_ticks_ >= kAlignStableTicksRequired) {
                    gimbal_.RequestYawZero();
                    gimbal_wait_align_then_reset_zero_ = false;
                    gimbal_align_stable_ticks_ = 0;
                }
            } else {
                gimbal_align_stable_ticks_ = 0;
            }
        }

        /********************** 底盘 ***********************/ 
        if(referee_.game_status_.game_type == GameType::RMUL_Infantry)
        {
            if (mcu_comm_data_local.Switch.fast_run == 1) {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_1V1_FAST_RUN / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_1V1_FAST_RUN / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_1V1_FAST_RUN / 128.0f));
            } else {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_1V1 / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_1V1 / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_1V1 / 128.0f));
            }
        }
        else if(referee_.game_status_.game_type == GameType::RMUL_3V3)
        {
            if (mcu_comm_data_local.Switch.fast_run == 1) {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_3V3_FAST_RUN / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_3V3_FAST_RUN / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_3V3_FAST_RUN / 128.0f));    
            } else {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * (CHASSIS_SPEED_3V3 ) / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * (CHASSIS_SPEED_3V3 ) / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * (CHASSIS_SPIN_SPEED_3V3 ) / 128.0f));
            }
        }
        else if(referee_.game_status_.game_type == GameType::RMUC)
        {
            if (referee_.status_.level <= 6) {
                if (mcu_comm_data_local.Switch.fast_run == 1) {
                    chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7_FAST_RUN / 128.0f);
                    chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7_FAST_RUN / 128.0f);
                    chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7_FAST_RUN / 128.0f));    
                } else {
                    chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7 / 128.0f);
                    chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7 / 128.0f);
                    chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7 / 128.0f));
                }
            } else {
                if (mcu_comm_data_local.Switch.fast_run == 1) {
                    chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7_FAST_RUN_HIGH / 128.0f);
                    chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7_FAST_RUN_HIGH / 128.0f);
                    chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH / 128.0f));    
                } else {
                    chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7_HIGH / 128.0f);
                    chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7_HIGH / 128.0f);
                    chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7_HIGH / 128.0f));
                }
            }
        }
        else {
            if (mcu_comm_data_local.Switch.fast_run == 1) {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7_FAST_RUN / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7_FAST_RUN / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7_FAST_RUN / 128.0f));    
            } else {
                chassis_.SetTargetVxInGimbal((mcu_comm_data_local.chassis_speed_x - 127.0f) * CHASSIS_SPEED_7V7 / 128.0f);
                chassis_.SetTargetVyInGimbal((127.0f - mcu_comm_data_local.chassis_speed_y ) * CHASSIS_SPEED_7V7 / 128.0f);
                chassis_.SetTargetVelocityRotation(((127.0f - mcu_comm_data_local.chassis_rotation ) * CHASSIS_SPIN_SPEED_7V7 / 128.0f));
            }
        }

        chassis_.SetYawAngle(-normalize_angle_pm_pi(gimbal_.GetNowYawAngle()/YAW_GEAR_RATIO));
        chassis_.SetPowerLimit(static_cast<float>(referee_.GetChassisPowerLimit()));
        chassis_.SetPowerBufferEnergy(static_cast<float>(referee_.GetChassisPowerBuffer()));


        /********************** 模式切换 ***********************/   
        switch(mcu_comm_data_local.chassis_spin)
        {
            case CHASSIS_SPIN_CLOCKWISE:
                // --- 缓启动逼近 ---
#ifndef VAR_SPIN_MODE
                if(referee_.game_status_.game_type == GameType::RMUL_Infantry)
                {
                    if (mcu_comm_data_local.Switch.fast_run == 1) {
                        if (spin_speed < CHASSIS_SPIN_SPEED_1V1_FAST_RUN) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_1V1_FAST_RUN);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_1V1_FAST_RUN) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_1V1_FAST_RUN);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_1V1_FAST_RUN);
                    } else {
                        if (spin_speed < CHASSIS_SPIN_SPEED_1V1) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_1V1);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_1V1) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_1V1);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_1V1);
                    }
                }
                else if (referee_.game_status_.game_type == GameType::RMUL_3V3) {
                    if (mcu_comm_data_local.Switch.fast_run == 1) {
                        if (spin_speed < CHASSIS_SPIN_SPEED_3V3_FAST_RUN) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_3V3_FAST_RUN);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_3V3_FAST_RUN) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_3V3_FAST_RUN);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_3V3_FAST_RUN);
                    } else {
                        if (spin_speed < CHASSIS_SPIN_SPEED_3V3) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_3V3);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_3V3) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_3V3);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_3V3);
                    }
                } else if (referee_.game_status_.game_type == GameType::RMUC) {
                    if (referee_.status_.level <= 6) {
                        if (mcu_comm_data_local.Switch.fast_run == 1) {
                            if (spin_speed < CHASSIS_SPIN_SPEED_7V7_FAST_RUN) {
                                spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                            } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7_FAST_RUN) {
                                spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                            }

                            gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                        } else {
                            if (spin_speed < CHASSIS_SPIN_SPEED_7V7) {
                                spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7);
                            } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7) {
                                spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7);
                            }

                            gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_7V7);
                        }
                    } else {
                        if (mcu_comm_data_local.Switch.fast_run == 1) {
                            if (spin_speed < CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH) {
                                spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH);
                            } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH) {
                                spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH);
                            }

                            gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_7V7_FAST_RUN_HIGH);
                        } else {
                            if (spin_speed < CHASSIS_SPIN_SPEED_7V7_HIGH) {
                                spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7_HIGH);
                            } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7_HIGH) {
                                spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7_HIGH);
                            }
                        }
                    }
                }else {
                    if (mcu_comm_data_local.Switch.fast_run == 1) {
                        if (spin_speed < CHASSIS_SPIN_SPEED_7V7_FAST_RUN) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7_FAST_RUN) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_7V7_FAST_RUN);
                    } else {
                        if (spin_speed < CHASSIS_SPIN_SPEED_7V7) {
                            spin_speed = fminf(spin_speed + accel, CHASSIS_SPIN_SPEED_7V7);
                        } else if (spin_speed > CHASSIS_SPIN_SPEED_7V7) {
                            spin_speed = fmaxf(spin_speed - accel, CHASSIS_SPIN_SPEED_7V7);
                        }

                        gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * CHASSIS_SPIN_SPEED_7V7);
                    }
                }
            
                chassis_.SetTargetVelocityRotation(spin_speed);
                gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_OMEGA);
                gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f)*YAW_SPEED_SENSITIVITY); //补偿速度可能符号错了

                referee_.spin_status_ = true;
#else
                // 变速小陀螺模式（编译选项 VAR_SPIN_MODE 打开）: 在基准转速附近按正弦周期变速
                {
                    float base_speed = CHASSIS_SPIN_SPEED_3V3;
                    if (referee_.game_status_.game_type == GameType::RMUL_Infantry) {
                        base_speed = (mcu_comm_data_local.Switch.fast_run == 1) ? CHASSIS_SPIN_SPEED_1V1_FAST_RUN : CHASSIS_SPIN_SPEED_1V1;
                    } else if (referee_.game_status_.game_type == GameType::RMUL_3V3) {
                        base_speed = (mcu_comm_data_local.Switch.fast_run == 1) ? CHASSIS_SPIN_SPEED_3V3_FAST_RUN : CHASSIS_SPIN_SPEED_3V3;
                    } else {
                        base_speed = (mcu_comm_data_local.Switch.fast_run == 1) ? CHASSIS_SPIN_SPEED_3V3_FAST_RUN : CHASSIS_SPIN_SPEED_3V3;
                    }

                    const float kTwoPi = 2.0f * PI;
                    const float sine = sinf(var_spin_phase);
                    const float target_ratio = var_spin_min_ratio + 0.5f * (1.0f + sine) * (var_spin_max_ratio - var_spin_min_ratio);
                    const float target_speed = target_ratio * base_speed;

                    // 直接采用正弦目标速度，无额外缓启动
                    spin_speed = target_speed;

                    var_spin_phase += var_spin_freq;
                    if (var_spin_phase >= kTwoPi) var_spin_phase -= kTwoPi;

                    gimbal_.SetYawOmegaFeedforword(YAW_FEEDFORWORD_RATIO * target_speed);
                    chassis_.SetTargetVelocityRotation(spin_speed);
                    gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_OMEGA);
                    gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f)*YAW_SPEED_SENSITIVITY);

                    referee_.spin_status_ = true;
                }
#endif
            break;
            case CHASSIS_SPIN_DISABLE:
                spin_speed = 0;
                // chassis_.SetTargetVelocityRotation((mcu_comm_data_local.chassis_rotation-127.0f)*35.0f/128.0f);
                // chassis_.SetTargetVelocityRotation(0.0f);
                gimbal_.SetGimbalYawControlType(GIMBAL_CONTROL_TYPE_ANGLE);
                gimbal_.SetYawOmegaFeedforword(0.0f);

                referee_.spin_status_ = false;
            break;
            case 2: // 疯车保护
                chassis_.Exit();
                gimbal_.Exit();

                referee_.spin_status_ = false;
            break;
            default:
            // do nothing
            break;
        };
        if (mcu_comm_data_local.Switch.reset_zero == 1)
        {
            gimbal_.RequestYawZero();
        }

        /********************** mini PC ***********************/  
         
        if(mcu_comm_data_local.Switch.auto_aim_flag == 1) {
            // if(fabs(mcu_comm_.mcu_autoaim_data_.pitch_angle) <0.3f){
            virtual_pitch_angle_ =  mcu_comm_.mcu_imu_data_.pitch_f
                                    - mcu_comm_.mcu_autoaim_data_.pitch_angle;
            minipc_pitch_recive_filter_.Update(virtual_pitch_angle_);
            virtual_pitch_angle_ = minipc_pitch_recive_filter_.GetOutput();

            virtual_yaw_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f
                                    + (mcu_comm_.mcu_autoaim_data_.yaw_angle);
            minipc_yaw_recive_filter_.Update(virtual_yaw_angle_);
            virtual_yaw_angle_ = minipc_yaw_recive_filter_.GetOutput();
   
        }
        // // 回传云台电机角度数据
        mcu_comm_.mcu_send_data_.yaw_angle = -gimbal_.GetYawNowAngleNoncumulative();
        mcu_comm_.mcu_send_data_.pitch_angle = -gimbal_.GetPitchNowAngleNoncumulative();
        mcu_comm_.mcu_send_data_.bullet_speed = referee_.GetInitialSpeed();
        mcu_comm_.CanSendCommand();
        
        
        /********************** 超级电容 ***********************/
        if (mcu_comm_data_local.Switch.supercap == 0) {
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
            referee_.supercap_status_ = false;
        } else if (mcu_comm_data_local.Switch.supercap == 1) {
            supercap_.SetChargeStatus(SUPERCAP_STATUS_DISCHARGE);
            referee_.supercap_status_ = true;
        } else {
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
            referee_.supercap_status_ = false;
        }
        supercap_.SetPowerLimitMax(100);
        supercap_.SetChargePower(50);
 
        /********************** 上板发来的状态 ***********************/
        referee_.booster_status_ = (mcu_comm_data_local.Switch.booster_status == 1);

        referee_.fastrun_status_ = (mcu_comm_data_local.Switch.fast_run == 1);
        
        if (mcu_comm_data_local.Switch.refresh_ui == 1){
            referee_.request_ui = true;
        }
        /********************** 调试信息 ***********************/   
        // debug_tools_.VofaSendFloat((mcu_comm_data_local.chassis_rotation-127.0f)*35.0f/128.0f);

        debug_tools_.VofaSendFloat(virtual_yaw_angle_);
        debug_tools_.VofaSendFloat(mcu_comm_.mcu_imu_data_.yaw_total_angle_f); 
        debug_tools_.VofaSendFloat(gimbal_.GetTargetYawOmega());
        debug_tools_.VofaSendFloat(gimbal_.GetImuYawOmega());
        debug_tools_.VofaSendFloat(gimbal_.GetFilteredImuYawOmega());
        // debug_tools_.VofaSendFloat(virtual_pitch_angle_);
        // debug_tools_.VofaSendFloat(mcu_comm_.mcu_imu_data_.pitch_f); 
        // debug_tools_.VofaSendFloat(referee_.GetInitialSpeed()); 
        // debug_tools_.VofaSendFloat(chassis_.GetPowerBufferConsume());
        // debug_tools_.VofaSendFloat(static_cast<float>(referee_.GetChassisPowerBuffer()));
        // debug_tools_.VofaSendFloat(chassis_.GetPowerScale());

        // debug_tools_.VofaSendFloat(gimbal_.GetYawNowAngleNoncumulative());
        // debug_tools_.VofaSendFloat(bmi088_.yaw_rad);
        // debug_tools_.VofaSendFloat(gimbal_.GetTargetPitchOmega());
        // debug_tools_.VofaSendFloat(gimbal_.GetNowPitchOmega());

        // 调试帧尾部
        debug_tools_.VofaSendTail();

        mcu_comm_data_local_pre = mcu_comm_data_local;

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
