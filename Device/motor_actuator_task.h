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

// New per-motor cmd/state topics
#include "../communication_topic/motor_topics.hpp"

#include "actuator/actuator_iface.hpp"

// Use minimal drivers directly (no adapters)
#include "actuator/drivers/dji_c6xx_min.hpp"
#include "actuator/drivers/dm_mit_min.hpp"

// 电机任务（新模型）：
// - 去掉 Role 概念
// - 上层发布 orb::motor_cmd（带 bus + std_id），任务按 id 路由到具体电机
// - CAN Rx 后发布 orb::motor_state（带 bus + std_id）
// - CAN Tx 仍走现有语义化 Tx Topics（dji_current_group_cmd / dm_tx_frame）+ CanMotorTxTask
//
// 说明：已完成上层迁移，MotorActuatorTask 仅消费 orb::motor_cmd / orb::motor_admin_cmd。
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
    BspCanHandle GetCanHandle(orb::MotorBus bus) const;

private:
    static void TaskEntry(void* arg);
    void Task();

    // id-based lookup (uses id_index_ + last-hit cache)
    actuator::IActuator* FindById(orb::MotorBus bus, uint16_t std_id);
    const actuator::Item* FindItemById(orb::MotorBus bus, uint16_t std_id) const;

    void PublishStateFor(orb::MotorBus bus, uint16_t std_id, const actuator::State& st);

    // unified Rx dispatch with small cache
    void OnCanRx(orb::MotorBus bus, const BspCanFrame* frame);
    uint8_t FindIndexByIdCached(orb::MotorBus bus, uint16_t std_id) const;

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
    actuator::IActuator* actuators_[kMaxActuators]{};

    // 快速索引：bus+std_id -> cfg/items/actuators_ index
    // 简单线性表（kMaxActuators 很小），并带 last-hit cache 减少热点查找开销
    struct IdIndex {
        orb::MotorBus bus;
        uint16_t std_id;
        uint8_t index;
    };
    IdIndex id_index_[kMaxActuators]{};
    uint8_t id_index_size_ = 0;
    mutable uint8_t last_hit_index_ = 0;

    // concrete storage (no heap) - minimal drivers
    actuator::drivers::DjiC6xxMin dji_impl_[6]{}; // up to 0x201..0x206 used in this project
    actuator::drivers::DmMitMin dm_impl_[4]{};

    // subs (new)
    RingSub<orb::MotorCmd, 32> motor_cmd_sub_{orb::motor_cmd};
    RingSub<orb::MotorAdminCmd, 16> motor_admin_sub_{orb::motor_admin_cmd};
};
