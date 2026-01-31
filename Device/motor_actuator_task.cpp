#include "motor_actuator_task.h"

#include "../communication_topic/motor_topics.hpp"
#include "../communication_topic/dm_motor_topics.hpp"

#include <cstring>

// (legacy) local helpers removed: task now uses cfg_.items[i].bus/std_id directly

uint8_t MotorActuatorTask::FindIndexByIdCached(orb::MotorBus bus, uint16_t std_id) const
{
    // last-hit cache
    if (last_hit_index_ < id_index_size_) {
        const auto& hit = id_index_[last_hit_index_];
        if (hit.bus == bus && hit.std_id == std_id) {
            return hit.index;
        }
    }

    for (uint8_t i = 0; i < id_index_size_; ++i) {
        const auto& e = id_index_[i];
        if (e.bus == bus && e.std_id == std_id) {
            last_hit_index_ = i;
            return e.index;
        }
    }

    return 0xFF;
}

void MotorActuatorTask::OnCanRx(orb::MotorBus bus, const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }

    if (frame->id_type != BSP_CAN_ID_STD) {
        return;
    }
    if (frame->frame_type != BSP_CAN_FRAME_DATA) {
        return;
    }
    if (frame->id > 0x7FFu) {
        return;
    }
    if (frame->len < 8u) {
        return;
    }

    const uint8_t idx = FindIndexByIdCached(bus, static_cast<uint16_t>(frame->id));
    if (idx == 0xFF) {
        return;
    }

    // Dispatch by type: minimal drivers expect raw payload
    const auto& item = cfg_.items[idx];
    switch (item.type) {
    case actuator::Type::DjiC620:
        if (actuators_[idx]) {
            // dji driver: OnRx(uint8_t[8])
            static_cast<actuator::drivers::DjiC6xxMin*>(actuators_[idx])->OnRx(frame->data);
            PublishStateFor(bus, static_cast<uint16_t>(item.std_id), actuators_[idx]->GetState());
        }
        break;
    case actuator::Type::DmNormal:
        if (actuators_[idx]) {
            static_cast<actuator::drivers::DmMitMin*>(actuators_[idx])->OnRx(frame->data);
            PublishStateFor(bus, static_cast<uint16_t>(item.std_id), actuators_[idx]->GetState());
        }
        break;
    default:
        break;
    }
}

actuator::IActuator* MotorActuatorTask::FindById(orb::MotorBus bus, uint16_t std_id)
{
    const uint8_t idx = FindIndexByIdCached(bus, std_id);
    if (idx == 0xFF) {
        return nullptr;
    }
    return actuators_[idx];
}

const actuator::Item* MotorActuatorTask::FindItemById(orb::MotorBus bus, uint16_t std_id) const
{
    const uint8_t idx = FindIndexByIdCached(bus, std_id);
    if (idx == 0xFF) {
        return nullptr;
    }
    return &cfg_.items[idx];
}

void MotorActuatorTask::PublishStateFor(orb::MotorBus bus, uint16_t std_id, const actuator::State& st)
{
    orb::MotorState s{};
    s.id.bus = bus;
    s.id.std_id = std_id;
    s.current = st.current;
    s.omega = st.omega;
    s.angle = st.angle;
    s.temperature = st.temperature;
    s.online = st.online;

    orb::motor_state.publish(s);
}

BspCanHandle MotorActuatorTask::GetCanHandle(orb::MotorBus bus) const
{
    switch (bus) {
    case orb::MotorBus::CAN1: return can1_;
    case orb::MotorBus::CAN2: return can2_;
    case orb::MotorBus::CAN3: return can3_;
    default: return nullptr;
    }
}

void MotorActuatorTask::Bind(BspCanHandle can1, BspCanHandle can2, BspCanHandle can3)
{
    configASSERT(!started_);
    can1_ = can1;
    can2_ = can2;
    can3_ = can3;
    bound_ = true;
}

void MotorActuatorTask::BindConfig(const motor_cfg::Config& cfg)
{
    configASSERT(!started_);
    cfg_ = cfg;
    cfg_bound_ = true;
}

bool MotorActuatorTask::Start() {
    if (started_) {
        configASSERT(false);
        return false;
    }
    configASSERT(bound_);
    configASSERT(cfg_bound_);
    configASSERT(can1_ != nullptr);
    configASSERT(can3_ != nullptr);

    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        actuators_[i] = nullptr;
    }

    id_index_size_ = 0;
    last_hit_index_ = 0;

    uint8_t dji_used = 0;
    uint8_t dm_used = 0;

    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        const auto& item = cfg_.items[i];

        // index entry for any valid (bus,std_id)
        if (item.type != actuator::Type::None && item.std_id != 0) {
            for (uint8_t j = 0; j < id_index_size_; ++j) {
                const auto& e = id_index_[j];
                if (e.bus == static_cast<orb::MotorBus>(item.bus) && e.std_id == static_cast<uint16_t>(item.std_id)) {
                    configASSERT(false);
                }
            }
            if (id_index_size_ < kMaxActuators) {
                id_index_[id_index_size_++] = IdIndex{static_cast<orb::MotorBus>(item.bus), static_cast<uint16_t>(item.std_id), i};
            }
        }

        // construct driver instance pointers
        switch (item.type) {
        case actuator::Type::DjiC620:
            if (dji_used < static_cast<uint8_t>(sizeof(dji_impl_) / sizeof(dji_impl_[0]))) {
                actuators_[i] = &dji_impl_[dji_used++];
            } else {
                configASSERT(false);
            }
            break;
        case actuator::Type::DmNormal:
            if (dm_used < static_cast<uint8_t>(sizeof(dm_impl_) / sizeof(dm_impl_[0]))) {
                actuators_[i] = &dm_impl_[dm_used++];
            } else {
                configASSERT(false);
            }
            break;
        case actuator::Type::None:
            break;
        default:
            configASSERT(false);
            break;
        }

        // bind/configure now (no second pass)
        if (!actuators_[i]) {
            continue;
        }

        BspCanHandle can = nullptr;
        switch (item.bus) {
        case actuator::Bus::CAN1: can = can1_; break;
        case actuator::Bus::CAN2: can = can2_; break;
        case actuator::Bus::CAN3: can = can3_; break;
        default: break;
        }
        configASSERT(can != nullptr);

        if (item.type == actuator::Type::DjiC620) {
            auto c = item.dji;
            c.bus = static_cast<orb::MotorBus>(item.bus);
            c.rx_std_id = static_cast<uint16_t>(item.std_id);
            static_cast<actuator::drivers::DjiC6xxMin*>(actuators_[i])->Init(can, c);
            static_cast<actuator::drivers::DjiC6xxMin*>(actuators_[i])->SetTargetOmega(0.0f);
        } else if (item.type == actuator::Type::DmNormal) {
            auto c = item.dm;
            c.bus = static_cast<orb::DmBus>(item.bus);
            c.can_rx_id = static_cast<uint8_t>(item.std_id & 0x0Fu);
            static_cast<actuator::drivers::DmMitMin*>(actuators_[i])->Init(can, c);
        }
    }

    // DM bring-up sequence (generic)
    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        const auto& it = cfg_.items[i];
        if (it.type != actuator::Type::DmNormal) continue;
        if (!actuators_[i]) continue;

        auto* dm = static_cast<actuator::drivers::DmMitMin*>(actuators_[i]);
        dm->ClearError();
        osDelay(cfg_.delay_clear_error_ms);
        dm->Enter();
        osDelay(cfg_.delay_enter_ms);
    }

    evt_attr_ = osEventFlagsAttr_t{
        .name = "motor_act_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    configASSERT(evt_ != nullptr);

    notifier_ = Notifier(evt_, kEvtBit);
    orb::motor_cmd.register_notifier(&notifier_);
    orb::motor_admin_cmd.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "motor_act",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityAboveNormal,
    };

    thread_ = osThreadNew(&MotorActuatorTask::TaskEntry, this, &attr);
    configASSERT(thread_ != nullptr);
    started_ = true;
    return true;
}

void MotorActuatorTask::Task() {
    for (;;) {
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        // 0) motor admin cmds (new API)
        {
            orb::MotorAdminCmd cmd{};
            while (motor_admin_sub_.copy(cmd)) {
                const uint8_t idx = FindIndexByIdCached(cmd.id.bus, cmd.id.std_id);
                if (idx == 0xFF) {
                    continue;
                }
                if (cfg_.items[idx].type != actuator::Type::DmNormal) {
                    continue;
                }
                if (!actuators_[idx]) {
                    continue;
                }

                auto* dm = static_cast<actuator::drivers::DmMitMin*>(actuators_[idx]);
                switch (cmd.op) {
                case orb::MotorAdminOp::Enter: dm->Enter(); break;
                case orb::MotorAdminOp::Exit: dm->Exit(); break;
                case orb::MotorAdminOp::ClearError: dm->ClearError(); break;
                case orb::MotorAdminOp::SaveZero: dm->SaveZero(); break;
                default: break;
                }
            }
        }

        // 1) handle motor_cmds (new API)
        {
            orb::MotorCmd cmd{};
            while (motor_cmd_sub_.copy(cmd)) {
                const uint8_t idx = FindIndexByIdCached(cmd.id.bus, cmd.id.std_id);
                if (idx == 0xFF) continue;
                if (!actuators_[idx]) continue;

                const auto& item = cfg_.items[idx];
                if (item.type == actuator::Type::DjiC620) {
                    auto* m = static_cast<actuator::drivers::DjiC6xxMin*>(actuators_[idx]);
                    if (cmd.mode == orb::MotorCtrlMode::Omega) {
                        m->SetTargetOmega(cmd.target_omega);
                    } else if (cmd.mode == orb::MotorCtrlMode::Current) {
                        m->SetTargetCurrent(cmd.target_current);
                    } else {
                        m->SetTargetOmega(0.0f);
                    }
                    m->Update();
                    m->PublishTx();
                } else if (item.type == actuator::Type::DmNormal) {
                    auto* m = static_cast<actuator::drivers::DmMitMin*>(actuators_[idx]);
                    // Only Angle/Omega/Torque are meaningful for MIT in this minimal layer
                    const float ang = (cmd.mode == orb::MotorCtrlMode::Angle) ? cmd.target_angle : m->now_angle();
                    const float omg = (cmd.mode == orb::MotorCtrlMode::Omega) ? cmd.target_omega : 0.0f;
                    const float tq  = (cmd.mode == orb::MotorCtrlMode::Current) ? cmd.target_current : 0.0f;
                    m->SetControl(ang, omg, tq);
                    m->PublishMitTx();
                }
            }
        }
    }
}

void MotorActuatorTask::TaskEntry(void* arg)
{
    auto* self = static_cast<MotorActuatorTask*>(arg);
    configASSERT(self != nullptr);
    self->Task();
}

void MotorActuatorTask::OnCan1Rx(const BspCanFrame* frame)
{
    OnCanRx(orb::MotorBus::CAN1, frame);
}

void MotorActuatorTask::OnCan2Rx(const BspCanFrame* frame)
{
    OnCanRx(orb::MotorBus::CAN2, frame);
}

void MotorActuatorTask::OnCan3Rx(const BspCanFrame* frame)
{
    OnCanRx(orb::MotorBus::CAN3, frame);
}
