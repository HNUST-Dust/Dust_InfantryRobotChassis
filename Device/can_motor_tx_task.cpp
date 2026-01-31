#include "can_motor_tx_task.h"

#include <cstring>

#include "../communication_topic/motor_topics.hpp"
#include "../communication_topic/dm_motor_topics.hpp"

namespace {
inline void pack_i16_be(uint8_t* p, int16_t v) {
    p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}
}  // namespace

bool CanMotorTxTask::Start() {
    if (started_) {
        configASSERT(false);
        return false;
    }
    started_ = true;

    // Platform 提供 HAL-free handle
    can1_ = bsp_can_get(BSP_CAN_BUS1);
    can2_ = bsp_can_get(BSP_CAN_BUS2);
    can3_ = bsp_can_get(BSP_CAN_BUS3);

    evt_attr_ = osEventFlagsAttr_t{
        .name = "can_motor_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    // Notifier：由 Topic 发布触发 osEventFlagsSet（ISR-safe）
    notifier_ = Notifier(evt_, kEvtBit);
    orb::dji_current_group_cmd.register_notifier(&notifier_);
    orb::dm_tx_frame.register_notifier(&notifier_);
    orb::dm_1to4_current_group_cmd.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "can_motor_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityHigh,
    };

    thread_ = osThreadNew(&CanMotorTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void CanMotorTxTask::TaskEntry(void* arg) {
    static_cast<CanMotorTxTask*>(arg)->Task();
}

void CanMotorTxTask::Task() {
    RingSub<orb::DjiCurrentGroupCmd, 8> dji_sub(orb::dji_current_group_cmd);
    RingSub<orb::DmTxFrame, 16> dm_sub(orb::dm_tx_frame);
    RingSub<orb::Dm1To4CurrentGroupCmd, 8> dm1to4_sub(orb::dm_1to4_current_group_cmd);

    for (;;) {
        // 事件驱动 + 兜底 1ms（避免 notifier 丢失导致卡死）
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        // 1) drain DJI 电流组帧
        {
            orb::DjiCurrentGroupCmd cmd{};
            while (dji_sub.copy(cmd)) {
                BspCanHandle h = nullptr;
                switch (cmd.bus) {
                    case orb::MotorBus::CAN1: h = can1_; break;
                    case orb::MotorBus::CAN2: h = can2_; break;
                    case orb::MotorBus::CAN3: h = can3_; break;
                    default: break;
                }
                if (!h) {
                    continue;
                }

                BspCanFrame f{};
                f.id = cmd.std_id;
                f.len = 8;
                f.id_type = BSP_CAN_ID_STD;
                f.frame_type = BSP_CAN_FRAME_DATA;
                f.is_fd = false;
                f.brs = false;

                std::memset(f.data, 0, 8);
                pack_i16_be(&f.data[0], cmd.current[0]);
                pack_i16_be(&f.data[2], cmd.current[1]);
                pack_i16_be(&f.data[4], cmd.current[2]);
                pack_i16_be(&f.data[6], cmd.current[3]);

                (void)bsp_can_send(h, &f);
            }
        }

        // 2) drain DM 发送帧
        {
            orb::DmTxFrame cmd{};
            while (dm_sub.copy(cmd)) {
                BspCanHandle h = nullptr;
                switch (cmd.bus) {
                    case orb::DmBus::CAN1: h = can1_; break;
                    case orb::DmBus::CAN2: h = can2_; break;
                    case orb::DmBus::CAN3: h = can3_; break;
                    default: break;
                }
                if (!h) {
                    continue;
                }

                BspCanFrame f{};
                f.id = cmd.std_id;
                f.len = cmd.len;
                f.id_type = BSP_CAN_ID_STD;
                f.frame_type = BSP_CAN_FRAME_DATA;
                f.is_fd = false;
                f.brs = false;

                std::memset(f.data, 0, 8);
                if (cmd.len > 0) {
                    std::memcpy(f.data, cmd.data, (cmd.len <= 8) ? cmd.len : 8);
                }

                (void)bsp_can_send(h, &f);
            }
        }

        // 3) drain DM 1-to-4 电流组帧
        {
            orb::Dm1To4CurrentGroupCmd cmd{};
            while (dm1to4_sub.copy(cmd)) {
                BspCanHandle h = nullptr;
                switch (cmd.bus) {
                    case orb::DmBus::CAN1: h = can1_; break;
                    case orb::DmBus::CAN2: h = can2_; break;
                    case orb::DmBus::CAN3: h = can3_; break;
                    default: break;
                }
                if (!h) {
                    continue;
                }

                BspCanFrame f{};
                f.id = cmd.std_id;
                f.len = 8;
                f.id_type = BSP_CAN_ID_STD;
                f.frame_type = BSP_CAN_FRAME_DATA;
                f.is_fd = false;
                f.brs = false;

                std::memset(f.data, 0, 8);
                pack_i16_be(&f.data[0], cmd.current[0]);
                pack_i16_be(&f.data[2], cmd.current[1]);
                pack_i16_be(&f.data[4], cmd.current[2]);
                pack_i16_be(&f.data[6], cmd.current[3]);

                (void)bsp_can_send(h, &f);
            }
        }
    }
}
