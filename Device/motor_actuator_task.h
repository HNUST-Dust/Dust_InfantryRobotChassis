/**
 * @file motor_actuator_task.h
 * @brief 执行器任务：把“业务层电机目标”映射为具体电机驱动与 CAN 报文。
 *
 * **定位**
 * - 设备层（Device）：面向“电机/执行器”这一类硬件对象。
 * - 上层（App/Interaction）不携带 CAN bus/std_id 等硬件细节，只发布 app-level motor topics。
 *
 * **数据流（Topic 化）**
 * - 输入：
 *   - `orb::chassis_wheel_omega_cmd`：底盘轮速目标（业务层）
 *   - `orb::gimbal_dm_target`：DM 目标（业务层）
 *   - `orb::gimbal_dm_admin_cmd`：DM 管理指令（进入/退出/清错等）
 * - 输出：
 *   - `orb::can_tx`：统一的 CAN 发送 Topic（由 Drivers/CanTxTask 作为唯一发送出口落到 BSP）
 *
 * **线程/回调模型**
 * - `Start()` 创建一个 RTOS 任务（静态栈/静态控制块，避免动态分配）。
 * - `OnCan{1,2,3}Rx()` 由 Platform/CAN RX 回调路径调用，可能处于中断或高优先级上下文：
 *   - 必须保持轻量、不可阻塞、不可调用会睡眠的 API。
 *   - 仅做帧过滤、驱动解包（`OnRx`）与在线 feed。
 *
 * **在线判据/守护**
 * - daemon_supervisor 的“在线”以收到预期电机反馈帧为判据；无反馈即认为执行器离线。
 * - fault 回调采取 best-effort：发布 `orb::gimbal_dm_admin_cmd` 请求 DM 退出。
 *
 * **约束**
 * - 严禁在业务层/设备层直接调用 `bsp_can_send*`；统一经由 `orb::can_tx` + CanTxTask。
 * - 必须先 `Bind()` + `BindConfig()`，再 `Start()`；启动后不允许重新绑定。
 */

#pragma once

#include "cmsis_os2.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
}

#include "bsp_can_port.h"

#include "motor_actuator_config.hpp"
#include "../communication_topic/topic_notify.hpp"

// CAN Tx topic (unified)
#include "../communication_topic/can_topics.hpp"

// app-level motor topics (no CAN info in upper layers)
#include "../communication_topic/app_motor_topics.hpp"

// Use minimal drivers directly (no adapters)
#include "actuator/drivers/dji_c6xx_min.hpp"
#include "actuator/drivers/dm_mit_min.hpp"

// 电机任务（精简版）：
// - 上层只发布 app-level motor topics（不包含 CAN bus/std_id）
// - 任务按配置映射到具体电机（电机初始化阶段已知道 bus/std_id）
// - CAN Tx 统一 publish 到 orb::can_tx，由 CanTxTask 发送
class MotorActuatorTask {
public:
    // 新接口：支持 CAN1/CAN2/CAN3
    void Bind(BspCanHandle can1, BspCanHandle can2, BspCanHandle can3);

    // 兼容旧接口：默认 can2=nullptr
    void Bind(BspCanHandle can1, BspCanHandle can3) { Bind(can1, nullptr, can3); }

    void BindConfig(const motor_cfg::Config& cfg);

    void OnCan1Rx(const BspCanFrame* frame);
    void OnCan2Rx(const BspCanFrame* frame);
    void OnCan3Rx(const BspCanFrame* frame);

    bool Start();

    // optional: expose bound CAN handles for diagnostics / reuse
    BspCanHandle GetCanHandle(orb::CanBus bus) const;

private:
    static void TaskEntry(void* arg);
    void Task();

    // unified Rx dispatch with small cache
    void OnCanRx(orb::CanBus bus, const BspCanFrame* frame);
    uint8_t FindIndexByIdCached(orb::CanBus bus, uint16_t std_id) const;

    bool started_ = false;
    osThreadId_t thread_ = nullptr;

    BspCanHandle can1_ = nullptr;
    BspCanHandle can2_ = nullptr;
    BspCanHandle can3_ = nullptr;
    bool bound_ = false;

    motor_cfg::Config cfg_{};
    bool cfg_bound_ = false;

    // event-driven
    osEventFlagsId_t evt_ = nullptr;
    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};
    static constexpr uint32_t kEvtBit = 1u << 0;
    Notifier notifier_{evt_, kEvtBit};

    StaticTask_t tcb_{};
    StackType_t stack_[768]{};

    static constexpr uint8_t kMaxActuators = motor_cfg::Config::kMaxActuators;

    // 快速索引：bus+std_id -> cfg/items/actuators_ index
    // 简单线性表（kMaxActuators 很小），并带 last-hit cache 减少热点查找开销
    struct IdIndex {
        orb::CanBus bus;
        uint16_t std_id;
        uint8_t index;
    };
    IdIndex id_index_[kMaxActuators]{};
    uint8_t id_index_size_ = 0;
    mutable uint8_t last_hit_index_ = 0;

    // concrete storage (no heap) - minimal drivers
    actuator::drivers::DjiC6xxMin dji_impl_[6]{}; // up to 0x201..0x206 used in this project
    actuator::drivers::DmMitMin dm_impl_[4]{};

    // per-item pointers (nullptr if the cfg slot is unused)
    actuator::drivers::DjiC6xxMin* dji_ptr_[kMaxActuators]{};
    actuator::drivers::DmMitMin* dm_ptr_[kMaxActuators]{};

    // app-level subs
    RingSub<orb::ChassisWheelOmegaCmd, 32> chassis_wheel_sub_{orb::chassis_wheel_omega_cmd};
    RingSub<orb::GimbalDmTarget, 16> gimbal_dm_target_sub_{orb::gimbal_dm_target};
    RingSub<orb::GimbalDmAdminCmd, 8> gimbal_dm_admin_sub_{orb::gimbal_dm_admin_cmd};
};

// Module singleton accessor (no Context/service-locator layer)
MotorActuatorTask& MotorActuatorTask_Instance();
