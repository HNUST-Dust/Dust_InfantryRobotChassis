#pragma once

#include "topic.hpp"

namespace orb {

// chassis spin 模式（强类型，避免依赖具体通信模块头文件）
enum class McuChassisSpinMode : uint8_t {
    Clockwise = 0,
    Disable = 1,
    CounterClockwise = 2,
};

// 超级电容用户命令（强类型，避免 0/1 魔法值）
enum class SupercapUserCmd : uint8_t {
    Charge = 0,
    Discharge = 1,
};

// 遥控/上板来的控制量（对应原 McuCommData）
struct McuControl {
    uint8_t yaw = 127;
    uint8_t pitch_angle = 127;
    uint8_t chassis_speed_x = 127;
    uint8_t chassis_speed_y = 127;
    uint8_t chassis_rotation = 127;
    McuChassisSpinMode chassis_spin = McuChassisSpinMode::Disable;
    SupercapUserCmd supercap = SupercapUserCmd::Charge;
    uint8_t auto_aim_flag = 0;
    uint8_t reset_zero = 0;
};

// 自瞄信息（对应 McuAutoaimData 的子集：当前工程只用 yaw_angle/pitch_angle）
struct McuAutoAim {
    float yaw_angle = 0.0f;
    float pitch_angle = 0.0f;
};

// IMU 信息（对应 McuImuData）
struct McuImu {
    float yaw_total_angle_f = 0.0f;
    float pitch_f = 0.0f;
    float yaw_omega_f = 0.0f;
    float pitch_omega_f = 0.0f;
};

// GIMBAL_INFO 对应的 CAN 标准帧 ID
constexpr uint16_t GIMBAL_INFO_ID = 0x0A;

struct GimbalInfoTx {
    float yaw_angle = 0.0f;
    float yaw_omega = 0.0f;
    float pitch_angle = 0.0f;
    float pitch_omega = 0.0f;
};

// Topic 实例：只保留最新值即可（Topic<T>）
inline Topic<McuControl> mcu_control;
inline Topic<McuAutoAim> mcu_autoaim;
inline Topic<McuImu> mcu_imu;
// 发送队列：缓冲最近几条（避免发布过快时丢失），并可注册 Notifier 实现事件唤醒
inline RingTopic<GimbalInfoTx, 4> gimbal_info_tx;

} // namespace orb
