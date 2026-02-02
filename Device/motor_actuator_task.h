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
