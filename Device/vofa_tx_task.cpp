#include "vofa_tx_task.h"

#include "../communication_topic/debug_topics.hpp"

bool VofaTxTask::Start(BspUartId uart_id) {
    if (started_) {
        configASSERT(false);
        return false;
    }
    started_ = true;

    uart_ = bsp_uart_get(uart_id);

    evt_attr_ = osEventFlagsAttr_t{
        .name = "vofa_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    // notifier_ 成员化，避免函数内 static 共享导致的多实例风险
    notifier_ = Notifier(evt_, kEvtBit);
    orb::vofa_tx.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "vofa_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityLow,
    };

    thread_ = osThreadNew(&VofaTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void VofaTxTask::TaskEntry(void* arg) {
    static_cast<VofaTxTask*>(arg)->Task();
}

void VofaTxTask::Task() {
    RingSub<orb::VofaTx, 64> sub(orb::vofa_tx);

    for (;;) {
        // 事件驱动 + 兜底 1ms（避免 notifier 丢失导致卡死）
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        if (uart_ == nullptr) {
            // UART 未就绪则丢弃（避免阻塞）
            orb::VofaTx tmp{};
            while (sub.copy(tmp)) {
                // drop
            }
            continue;
        }

        orb::VofaTx pkt{};
        while (sub.copy(pkt)) {
            (void)bsp_uart_send(uart_, const_cast<uint8_t*>(pkt.bytes), 4);
        }
    }
}
