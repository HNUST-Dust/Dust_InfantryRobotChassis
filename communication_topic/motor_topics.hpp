#pragma once

#include "topic.hpp"

// 电机相关 Topic：将控制（cmd）与状态（state）解耦。
//
// 新方案：去掉 Role 概念，改为“按电机ID/句柄”订阅/发布：
// - 每个电机只订阅自己的 cmd（电流/速度/位置）
// - 每个电机只发布自己的 state（电流/速度/位置/在线）
// - 管理类命令（置零/使能/失能/清错）也通过 MotorId 下发

namespace orb {

enum class MotorBus : uint8_t {
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
};

// ===== 单电机通用命令/状态（推荐新接口） =====

enum class MotorCtrlMode : uint8_t {
    Disabled = 0,
    Current,
    Omega,
    Angle,
};

struct MotorId {
    MotorBus bus = MotorBus::CAN1;
    uint16_t std_id = 0;   // CAN StdId (e.g. DJI 0x201 / DM 0x12)
};

struct MotorCmd {
    MotorId id{};
    MotorCtrlMode mode = MotorCtrlMode::Disabled;

    // 目标值（按 mode 使用）
    float target_current = 0.0f;
    float target_omega = 0.0f;
    float target_angle = 0.0f;
};

struct MotorState {
    MotorId id{};

    float current = 0.0f;
    float omega = 0.0f;
    float angle = 0.0f;

    float temperature = 0.0f;
    bool online = false;
};

// 命令总线：上层发布 MotorCmd，下层(电机/驱动任务)按 id 过滤并执行
inline RingTopic<MotorCmd, 32> motor_cmd{};

// 状态总线：电机发布 MotorState，上层按 id 过滤使用
inline RingTopic<MotorState, 32> motor_state{};

// ===== 管理/维护指令（推荐新接口） =====

enum class MotorAdminOp : uint8_t {
    None = 0,

    // 通用
    Enter,
    Exit,
    ClearError,

    // DM normal supports save zero
    SaveZero,
};

struct MotorAdminCmd {
    MotorId id{};
    MotorAdminOp op = MotorAdminOp::None;
};

inline RingTopic<MotorAdminCmd, 16> motor_admin_cmd{};

// ===== 旧接口：DJI 0x200/0x1FF/0x2FF 四路电流组帧（暂保留兼容） =====

// DJI C6xx (0x200) 组帧：四个电机电流/力矩输出（16-bit 有符号）
// 对应标准帧 0x200 / 0x1FF / 0x2FF 的 payload 中的 4 路。
struct DjiCurrentGroupCmd {
    MotorBus bus = MotorBus::CAN1;
    uint16_t std_id = 0x200;       // 0x200/0x1FF/0x2FF
    int16_t current[4] = {0, 0, 0, 0};
};

// 发送队列：发布即唤醒 CAN TxTask
inline RingTopic<DjiCurrentGroupCmd, 8> dji_current_group_cmd;

} // namespace orb
