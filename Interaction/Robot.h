#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
// app
#include "app_chassis.h"
#include "app_gimbal.h"
#include "user_lib.h"
#include "low_pass_filter.hpp"
// module
#include "dvc_MCU_comm.h"
#include "supercap.h"
#include "debug_tools.h"
#include "dvc_referee.h"
#include <stdint.h>

#define YAW_SENSITIVITY                  0.00008F//0.00008
#define YAW_SENSITIVITY_USED_IMU         0.00800F//0.00008
#define YAW_SPEED_SENSITIVITY            0.05f
#define PITCH_RANGE_MAX                  0.3f
#define PITCH_RANGE_MAX_USE_IMU          15.0f

#define CHASSIS_SPEED                    10.0f
#define CHASSIS_SPIN_SPEED               20.0f

#define YAW_GEAR_RATIO                   0.8f
#define YAW_FEEDFORWORD_RATIO            0.19f

/**
 * @brief 小陀螺类型
 * 
 */
enum RobotGyroscopeType
{
    ROBOT_GYROSCOPE_TYPE_DISABLE = 0,
    ROBOT_GYROSCOPE_TYPE_CLOCKWISE,
    ROBOT_GYROSCOPE_TYPE_COUNTERCLOCKWISE,  
};

class Robot
{
public:
    // 调试工具
    DebugTools debug_tools_;
    // 与上板的通讯服务
    McuComm mcu_comm_;
    // 底盘跟随控制PID
    Pid chassis_follow_pid_;
    // 底盘
    Chassis chassis_;
    // 底盘小陀螺斜坡规划器
    float ramp_temp = 0.0f;
    ramp_function_source_t chassis_spin_ramp_source;
    LowPassFilter minipc_recive_filter_;

    // 云台
    Gimbal  gimbal_;
    // 超级电容模组
    Supercap supercap_;
    // 底盘陀螺仪
    Imu imu_;
    // 裁判系统
    Referee referee_;

    void Init();
    void Task();
protected:
    // 机器人本场比赛的实时信息
    uint8_t id_;
    uint8_t level_;
    uint16_t current_hp_;
    uint16_t max_hp_;
    uint16_t shooter_barrel_cooling_value_;
    uint16_t shooter_barrel_heat_limit_; 
    uint16_t chassis_power_limit_;

    // 操作手控制的的虚拟角度
    float virtual_yaw_angle_ = 0;
    float virtual_pitch_angle_ = 0;
    
    // 小陀螺功能状态
    RobotGyroscopeType chassis_gyroscope_mode_status_ = ROBOT_GYROSCOPE_TYPE_DISABLE;
    // 底盘跟随模式是否使能
    bool chassis_follow_mode_status_ = true;
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);
};

#endif // !APP_ROBOT_H_