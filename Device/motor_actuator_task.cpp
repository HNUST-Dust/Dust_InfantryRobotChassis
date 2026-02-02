/**
 * @file motor_actuator_task.cpp
 * @brief MotorActuatorTask 实现：配置驱动实例、CAN RX 分发、以及 Topic 化 CAN TX 输出。
 *
 * 说明：
 * - 本文件只负责“电机任务”内部实现细节（驱动实例存储、id->index 缓存、RTOS 事件驱动）。
 * - CAN 的实际发送不在这里直接落到 BSP，而是发布 `orb::can_tx`，由 Drivers/CanTxTask 统一发送。
 *
 * 守护策略：
 * - Online 判据：收到匹配 (bus,std_id) 的反馈帧（且帧类型/长度合法）。
 * - Offline/Fault：daemon 回调发布管理指令请求 DM 退出（best-effort）。
 */

#include "motor_actuator_task.h"

#include "../communication_topic/can_topics.hpp"

#include <cstring>

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

void motor_actuator_daemon_fault(DaemonClient&)
{
    // Best-effort: request DM motors to Exit.
    orb::GimbalDmAdminCmd admin{};
    admin.op = orb::GimbalDmAdminOp::BothExit;
    orb::gimbal_dm_admin_cmd.publish(admin);
}

DaemonClient* s_motor_actuator_daemon = nullptr;
} // namespace

MotorActuatorTask& MotorActuatorTask_Instance()
{
    static MotorActuatorTask inst;
    return inst;
}

// (legacy) local helpers removed: task now uses cfg_.items[i].bus/std_id directly

uint8_t MotorActuatorTask::FindIndexByIdCached(orb::CanBus bus, uint16_t std_id) const
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

void MotorActuatorTask::OnCanRx(orb::CanBus bus, const BspCanFrame* frame)
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

    // Online criterion: receiving expected motor feedback frames.
    if (s_motor_actuator_daemon) {
        s_motor_actuator_daemon->feed(now_ms());
    }

    // Dispatch by type: minimal drivers expect raw payload
    const auto& item = cfg_.items[idx];
    switch (item.type) {
    case motor_cfg::Type::DjiC620:
        if (dji_ptr_[idx]) {
            dji_ptr_[idx]->OnRx(frame->data);
        }
        break;
    case motor_cfg::Type::DmNormal:
        if (dm_ptr_[idx]) {
            dm_ptr_[idx]->OnRx(frame->data);
        }
        break;
    default:
        break;
    }
}

BspCanHandle MotorActuatorTask::GetCanHandle(orb::CanBus bus) const
{
    switch (bus) {
    case orb::CanBus::CAN1: return can1_;
    case orb::CanBus::CAN2: return can2_;
    case orb::CanBus::CAN3: return can3_;
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

    // Actuator layer is core: monitor as CRITICAL and treat offline as FATAL.
    {
        static DaemonClient daemon(
            200,
            motor_actuator_daemon_fault,
            this,
            DaemonClient::Domain::CONTROL,
            DaemonClient::FaultLevel::FATAL,
            DaemonClient::Priority::CRITICAL);
        s_motor_actuator_daemon = &daemon;
        (void)DaemonSupervisor::register_client(s_motor_actuator_daemon);
        s_motor_actuator_daemon->feed(now_ms());
    }

    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        dji_ptr_[i] = nullptr;
        dm_ptr_[i] = nullptr;
    }

    id_index_size_ = 0;
    last_hit_index_ = 0;

    uint8_t dji_used = 0;
    uint8_t dm_used = 0;

    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        const auto& item = cfg_.items[i];

        // index entry for any valid (bus,std_id)
        if (item.type != motor_cfg::Type::None && item.std_id != 0) {
            for (uint8_t j = 0; j < id_index_size_; ++j) {
                const auto& e = id_index_[j];
                if (e.bus == item.bus && e.std_id == item.std_id) {
                    configASSERT(false);
                }
            }
            if (id_index_size_ < kMaxActuators) {
                id_index_[id_index_size_++] = IdIndex{item.bus, item.std_id, i};
            }
        }

        // construct driver instance pointers
        switch (item.type) {
        case motor_cfg::Type::DjiC620:
            if (dji_used < static_cast<uint8_t>(sizeof(dji_impl_) / sizeof(dji_impl_[0]))) {
                dji_ptr_[i] = &dji_impl_[dji_used++];
            } else {
                configASSERT(false);
            }
            break;
        case motor_cfg::Type::DmNormal:
            if (dm_used < static_cast<uint8_t>(sizeof(dm_impl_) / sizeof(dm_impl_[0]))) {
                dm_ptr_[i] = &dm_impl_[dm_used++];
            } else {
                configASSERT(false);
            }
            break;
        case motor_cfg::Type::None:
            break;
        default:
            configASSERT(false);
            break;
        }

        // bind/configure now (no second pass)
        if (!dji_ptr_[i] && !dm_ptr_[i]) {
            continue;
        }

        BspCanHandle can = GetCanHandle(item.bus);
        configASSERT(can != nullptr);

        if (item.type == motor_cfg::Type::DjiC620) {
            auto c = item.dji;
            c.bus = item.bus;
            c.rx_std_id = item.std_id;
            dji_ptr_[i]->Init(can, c);
            dji_ptr_[i]->SetTargetOmega(0.0f);
        } else if (item.type == motor_cfg::Type::DmNormal) {
            auto c = item.dm;
            c.bus = item.bus;
            c.can_rx_id = static_cast<uint8_t>(item.std_id & 0x0Fu);
            dm_ptr_[i]->Init(can, c);
        }
    }

    // DM bring-up sequence (generic)
    for (uint8_t i = 0; i < kMaxActuators; ++i) {
        const auto& it = cfg_.items[i];
        if (it.type != motor_cfg::Type::DmNormal) continue;
        if (!dm_ptr_[i]) continue;

        auto* dm = dm_ptr_[i];
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
    orb::chassis_wheel_omega_cmd.register_notifier(&notifier_);
    orb::gimbal_dm_target.register_notifier(&notifier_);
    orb::gimbal_dm_admin_cmd.register_notifier(&notifier_);

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

        // A) app-level chassis wheel omega cmds
        {
            orb::ChassisWheelOmegaCmd cmd{};
            while (chassis_wheel_sub_.copy(cmd)) {
                if (cmd.wheel >= 4) {
                    continue;
                }
                const uint8_t item_index = cfg_.chassis_wheel_index[cmd.wheel];
                if (item_index >= kMaxActuators) {
                    continue;
                }
                if (cfg_.items[item_index].type != motor_cfg::Type::DjiC620) {
                    continue;
                }
                if (!dji_ptr_[item_index]) {
                    continue;
                }

                dji_ptr_[item_index]->SetTargetOmega(cmd.omega);
                dji_ptr_[item_index]->Update();
                // DJI 组帧的实际 publish 在后续统一完成（避免单电机 publish 把其他电机清零）。
            }
        }

        // B) app-level gimbal targets
        {
            orb::GimbalDmTarget t{};
            while (gimbal_dm_target_sub_.copy(t)) {
                const uint8_t yaw_idx = cfg_.gimbal_yaw_index;
                if (yaw_idx < kMaxActuators && cfg_.items[yaw_idx].type == motor_cfg::Type::DmNormal && dm_ptr_[yaw_idx]) {
                    dm_ptr_[yaw_idx]->SetControl(t.yaw_angle, t.yaw_omega, t.yaw_torque);
                    dm_ptr_[yaw_idx]->PublishMitTx(t.kp, t.kd);
                }

                const uint8_t pit_idx = cfg_.gimbal_pitch_index;
                if (pit_idx < kMaxActuators && cfg_.items[pit_idx].type == motor_cfg::Type::DmNormal && dm_ptr_[pit_idx]) {
                    dm_ptr_[pit_idx]->SetControl(t.pitch_angle, t.pitch_omega, t.pitch_torque);
                    dm_ptr_[pit_idx]->PublishMitTx(t.kp, t.kd);
                }
            }
        }

        // C) app-level gimbal admin
        {
            orb::GimbalDmAdminCmd c{};
            while (gimbal_dm_admin_sub_.copy(c)) {
                const uint8_t yaw_idx = cfg_.gimbal_yaw_index;
                const uint8_t pit_idx = cfg_.gimbal_pitch_index;

                auto* yaw = (yaw_idx < kMaxActuators) ? dm_ptr_[yaw_idx] : nullptr;
                auto* pit = (pit_idx < kMaxActuators) ? dm_ptr_[pit_idx] : nullptr;

                switch (c.op) {
                case orb::GimbalDmAdminOp::YawEnter: if (yaw) yaw->Enter(); break;
                case orb::GimbalDmAdminOp::YawExit: if (yaw) yaw->Exit(); break;
                case orb::GimbalDmAdminOp::YawClearError: if (yaw) yaw->ClearError(); break;
                case orb::GimbalDmAdminOp::YawSaveZero: if (yaw) yaw->SaveZero(); break;
                case orb::GimbalDmAdminOp::PitchEnter: if (pit) pit->Enter(); break;
                case orb::GimbalDmAdminOp::PitchExit: if (pit) pit->Exit(); break;
                case orb::GimbalDmAdminOp::PitchClearError: if (pit) pit->ClearError(); break;
                case orb::GimbalDmAdminOp::BothEnter: if (yaw) yaw->Enter(); if (pit) pit->Enter(); break;
                case orb::GimbalDmAdminOp::BothExit: if (yaw) yaw->Exit(); if (pit) pit->Exit(); break;
                case orb::GimbalDmAdminOp::BothClearError: if (yaw) yaw->ClearError(); if (pit) pit->ClearError(); break;
                default: break;
                }
            }
        }

        // D) publish DJI group frames (0x200/0x1FF/0x2FF)
        {
            auto pack_i16_be = [](uint8_t* p, int16_t v) {
                p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
                p[1] = static_cast<uint8_t>(v & 0xFF);
            };

            struct GroupAcc {
                orb::CanBus bus;
                uint16_t group_std_id;
                int16_t current[4];
                bool used;
            } groups[8]{};

            auto slot_from_ids = [](uint16_t group_id, uint16_t motor_id) -> int {
                if (group_id == 0x200) {
                    if (motor_id >= 0x201 && motor_id <= 0x204) return static_cast<int>(motor_id - 0x201);
                } else if (group_id == 0x1FF) {
                    if (motor_id >= 0x205 && motor_id <= 0x208) return static_cast<int>(motor_id - 0x205);
                } else if (group_id == 0x2FF) {
                    if (motor_id >= 0x209 && motor_id <= 0x20C) return static_cast<int>(motor_id - 0x209);
                }
                // fallback by motor id range
                if (motor_id >= 0x201 && motor_id <= 0x204) return static_cast<int>(motor_id - 0x201);
                if (motor_id >= 0x205 && motor_id <= 0x208) return static_cast<int>(motor_id - 0x205);
                if (motor_id >= 0x209 && motor_id <= 0x20C) return static_cast<int>(motor_id - 0x209);
                return -1;
            };

            auto find_or_add_group = [&](orb::CanBus bus, uint16_t group_id) -> GroupAcc* {
                for (auto& g : groups) {
                    if (g.used && g.bus == bus && g.group_std_id == group_id) {
                        return &g;
                    }
                }
                for (auto& g : groups) {
                    if (!g.used) {
                        g.used = true;
                        g.bus = bus;
                        g.group_std_id = group_id;
                        g.current[0] = 0;
                        g.current[1] = 0;
                        g.current[2] = 0;
                        g.current[3] = 0;
                        return &g;
                    }
                }
                return nullptr;
            };

            for (uint8_t i = 0; i < kMaxActuators; ++i) {
                const auto& it = cfg_.items[i];
                if (it.type != motor_cfg::Type::DjiC620) {
                    continue;
                }
                if (!dji_ptr_[i]) {
                    continue;
                }

                const uint16_t group_id = it.dji_group_std_id;
                const int slot = slot_from_ids(group_id, it.std_id);
                if (slot < 0 || slot > 3) {
                    continue;
                }

                auto* g = find_or_add_group(it.bus, group_id);
                if (!g) {
                    continue;
                }
                g->current[slot] = dji_ptr_[i]->target_current_raw();
            }

            for (const auto& g : groups) {
                if (!g.used) {
                    continue;
                }

                orb::CanTxFrame out{};
                out.bus = g.bus;
                out.id = g.group_std_id;
                out.id_type = orb::CanIdType::Std;
                out.frame_type = orb::CanFrameType::Data;
                out.is_fd = false;
                out.brs = false;
                out.len = 8;
                std::memset(out.data, 0, sizeof(out.data));
                pack_i16_be(&out.data[0], g.current[0]);
                pack_i16_be(&out.data[2], g.current[1]);
                pack_i16_be(&out.data[4], g.current[2]);
                pack_i16_be(&out.data[6], g.current[3]);
                orb::can_tx.publish(out);
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
    OnCanRx(orb::CanBus::CAN1, frame);
}

void MotorActuatorTask::OnCan2Rx(const BspCanFrame* frame)
{
    OnCanRx(orb::CanBus::CAN2, frame);
}

void MotorActuatorTask::OnCan3Rx(const BspCanFrame* frame)
{
    OnCanRx(orb::CanBus::CAN3, frame);
}
