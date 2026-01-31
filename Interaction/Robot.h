#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_

// algo / common types
#include "alg_pid.h"       // Pid（当前虽未使用，但保留占位）
#include "low_pass_filter.hpp"

#include <stdint.h>
#include "cmsis_os2.h"

#define YAW_SENSITIVITY                  0.00008F//0.00008
#define YAW_SENSITIVITY_USED_IMU         0.00800F//0.00008
#define YAW_SPEED_SENSITIVITY            0.05f
#define PITCH_RANGE_MAX                  0.3f
#define PITCH_RANGE_MAX_USE_IMU          15.0f

#define CHASSIS_SPEED                    10.0f
#define CHASSIS_SPIN_SPEED               30.0f

#define YAW_GEAR_RATIO                   0.8f
#define YAW_FEEDFORWORD_RATIO            0.19f

/**
 * @brief 小陀螺类型（legacy，占位）
 */
enum class RobotGyroscopeType : uint8_t
{
    Disable = 0,
    Clockwise,
    Counterclockwise,
};

class Robot
{
public:
    // 融合与发布：Robot 不再直接持有/驱动具体模块对象（Chassis/Gimbal/Supercap/Referee 等）
    // 也不再依赖 HAL 句柄。

    // 底盘跟随控制PID（当前 Robot 未使用；保留作为参数占位，后续如要实现“跟随”可继续用）
    Pid chassis_follow_pid_;

    // miniPC 角度接收滤波
    LowPassFilter minipc_yaw_recive_filter_;
    LowPassFilter minipc_pitch_recive_filter_;

    // 过去的 Init() 主要做“创建任务 + 参数默认值/滤波器配置”。
    // 为了减少显式 Init 的样板，这里改为：对象构造后处于可用状态；需要 RTOS 任务时显式 Start()。
    void Start();
    void Task();
protected:
    // 机器人的比赛信息（当前 Robot 未直接使用；保留字段避免影响上层）
    uint8_t id_;
    uint8_t level_;
    uint16_t current_hp_;
    uint16_t max_hp_;
    uint16_t shooter_barrel_cooling_value_;
    uint16_t shooter_barrel_heat_limit_;
    uint16_t chassis_power_limit_;

    // 操作手控制的虚拟角度（Robot 融合输出）
    float virtual_yaw_angle_ = 0;
    float virtual_pitch_angle_ = 0;

    // legacy 状态位（当前 Robot 未直接使用；如后续要做模式机可继续沿用）
    RobotGyroscopeType chassis_gyroscope_mode_status_ = RobotGyroscopeType::Disable;
    bool chassis_follow_mode_status_ = true;
    int32_t robot_level_ = 1;

    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    static void TaskEntry(void *param);
};

#endif // !APP_ROBOT_H_