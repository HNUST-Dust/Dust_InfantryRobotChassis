#include "ui_tx_task.h"

#include "../communication_topic/ui_topics.hpp"

bool UiTxTask::Start(BspUartId uart_id) {
    if (started_) {
        configASSERT(false);
        return false;
    }
    started_ = true;

    uart_ = bsp_uart_get(uart_id);

    evt_attr_ = osEventFlagsAttr_t{
        .name = "ui_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    notifier_ = Notifier(evt_, kEvtBit);
    orb::ui_tx.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "ui_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityLow,
    };

    thread_ = osThreadNew(&UiTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void UiTxTask::TaskEntry(void* arg) {
    static_cast<UiTxTask*>(arg)->Task();
}

void UiTxTask::Task() {
    RingSub<orb::UiTxFrame, 16> sub(orb::ui_tx);

    for (;;) {
        // 事件驱动 + 兜底 1ms（避免 notifier 丢失导致卡死）
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        if (uart_ == nullptr) {
            // UART 未就绪则丢弃（避免阻塞）
            orb::UiTxFrame tmp{};
            while (sub.copy(tmp)) {
                // drop
            }
            continue;
        }

        bool sent_any = false;
        orb::UiTxFrame pkt{};
        while (sub.copy(pkt)) {
            if (pkt.len == 0 || pkt.len > sizeof(pkt.bytes)) {
                continue;
            }
            (void)bsp_uart_send(uart_, pkt.bytes, pkt.len);
            sent_any = true;

            // 固定节流：裁判系统 UI 协议发送需要限频，避免占满链路
            osDelay(10);
        }

        // 若本轮没有发送任何帧，避免空转（让出 CPU）
        if (!sent_any) {
            osDelay(1);
        }
    }
}
