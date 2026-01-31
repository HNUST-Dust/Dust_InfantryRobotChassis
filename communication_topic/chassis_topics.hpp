#pragma once

#include "topic.hpp"

// Robot <-> Chassis 解耦 Topic
// - Robot 发布 ChassisCmd（速度/旋转/偏航角等输入）
// - Chassis 订阅并驱动自身控制

namespace orb {

struct ChassisCmd {
    // 目标速度（云台坐标系）
    float vx_in_gimbal = 0.0f;
    float vy_in_gimbal = 0.0f;

    // 目标旋转速度（底盘自转）
    float v_rotation = 0.0f;

    // 云台相对底盘偏航角（用于坐标变换）
    float yaw_angle = 0.0f;

    // 命令：紧急退出（疯车保护）
    bool request_exit = false;
};

inline Topic<ChassisCmd> chassis_cmd;

}  // namespace orb
