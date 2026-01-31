#pragma once

#include "cmsis_os2.h"

#include "bsp_uart_port.h"
#include "../communication_topic/topic_notify.hpp"

// Forward decl for FreeRTOS static task storage types
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace orb {
struct VofaTx;
}

template<typename T, int DEPTH>
class RingSub;

template<typename T, int DEPTH>
class RingTopic;

// VOFA 发送任务：订阅 vofa_tx ring topic，串行发送到 UART7，避免控制环 busy-wait。
class VofaTxTask {
public:
    bool Start(BspUartId uart_id = BSP_UART7);

    // Backward-compatible name
    void Init(BspUartId uart_id = BSP_UART7) { (void)Start(uart_id); }

private:
    static void TaskEntry(void* arg);
    void Task();

    bool started_ = false;

    BspUartHandle uart_ = nullptr;

    osThreadId_t thread_ = nullptr;

    osEventFlagsId_t evt_ = nullptr;
    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};

    static constexpr uint32_t kEvtBit = 1u << 0;

    // 资源成员化，避免 Init() 内 static 导致的共享/覆盖风险
    Notifier notifier_{nullptr, 0};

    StaticTask_t tcb_{};
    StackType_t stack_[256]{};
};
