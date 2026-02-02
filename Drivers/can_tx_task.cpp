/**
 * @file can_tx_task.cpp
 * @brief `Drivers/can_tx_task.h` 的实现（drain `orb::can_tx` 并调用 `bsp_can_send()`）
 *
 * 实现要点：
 * =========
 * - 使用 `RingSub<orb::CanTxFrame, 32>` 作为订阅端，循环 drain 直至队列为空。
 * - 根据 `orb::CanBus` 选择对应 `BspCanHandle`（CAN1/2/3）。
 * - 发送时将 Topic 帧字段映射到 BSP 帧字段（STD/EXT、DATA/REMOTE、FD/BRS）。
 */

#include "can_tx_task.h"

#include <cstring>

#include "../communication_topic/can_topics.hpp"

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

DaemonClient* s_can_tx_daemon = nullptr;
} // namespace

namespace {
inline BspCanHandle pick_handle(orb::CanBus bus, BspCanHandle can1, BspCanHandle can2, BspCanHandle can3) {
    switch (bus) {
        case orb::CanBus::CAN1: return can1;
        case orb::CanBus::CAN2: return can2;
        case orb::CanBus::CAN3: return can3;
        default: return nullptr;
    }
}

inline BspCanIdType to_bsp_id_type(orb::CanIdType t) {
    return (t == orb::CanIdType::Ext) ? BSP_CAN_ID_EXT : BSP_CAN_ID_STD;
}

inline BspCanFrameType to_bsp_frame_type(orb::CanFrameType t) {
    return (t == orb::CanFrameType::Remote) ? BSP_CAN_FRAME_REMOTE : BSP_CAN_FRAME_DATA;
}
}  // namespace

bool CanTxTask::Start() {
    if (started_) {
        configASSERT(false);
        return false;
    }

    // Tx tasks don't receive external data; monitor them in a best-effort way.
    // Feed is driven by actual TX activity to avoid masking a stuck drain loop.
    {
        static DaemonClient s_daemon(
            3000,
            nullptr,
            this,
            DaemonClient::Domain::COMM,
            DaemonClient::FaultLevel::WARN,
            DaemonClient::Priority::HIGH);
        s_can_tx_daemon = &s_daemon;
        (void)DaemonSupervisor::register_client(s_can_tx_daemon);
        s_can_tx_daemon->feed(now_ms());
    }

    started_ = true;

    can1_ = bsp_can_get(BSP_CAN_BUS1);
    can2_ = bsp_can_get(BSP_CAN_BUS2);
    can3_ = bsp_can_get(BSP_CAN_BUS3);

    evt_attr_ = osEventFlagsAttr_t{
        .name = "can_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    notifier_ = Notifier(evt_, kEvtBit);
    orb::can_tx.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "can_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityHigh,
    };

    thread_ = osThreadNew(&CanTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void CanTxTask::TaskEntry(void* arg) {
    static_cast<CanTxTask*>(arg)->Task();
}

void CanTxTask::Task() {
    RingSub<orb::CanTxFrame, 32> can_sub(orb::can_tx);

    for (;;) {
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        // A) drain generic CAN frames
        {
            orb::CanTxFrame cmd{};
            while (can_sub.copy(cmd)) {
                BspCanHandle h = pick_handle(cmd.bus, can1_, can2_, can3_);
                if (!h) {
                    continue;
                }

                if (s_can_tx_daemon) {
                    s_can_tx_daemon->feed(now_ms());
                }

                BspCanFrame f{};
                f.id = cmd.id;
                f.len = cmd.len;
                std::memcpy(f.data, cmd.data, sizeof(f.data));
                f.id_type = to_bsp_id_type(cmd.id_type);
                f.frame_type = to_bsp_frame_type(cmd.frame_type);
                f.is_fd = cmd.is_fd;
                f.brs = cmd.brs;

                (void)bsp_can_send(h, &f);
            }
        }
    }
}
