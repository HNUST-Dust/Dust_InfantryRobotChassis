/**
 * @file uart_tx_task.h
 * @brief 统一 UART 发送任务（Topic -> BSP UART）
 *
 * 设计思路：
 * =========
 * - 订阅 `orb::uart_tx`（通用字节流 Topic），将上层准备好的字节序列发送到指定 UART 端口。
 * - 通过 Notifier/EventFlags 在有新数据时唤醒，避免无意义的高频轮询。
 * - 该任务是全仓 UART 发送的唯一出口：业务层禁止直接调用 `bsp_uart_send()`。
 *
 * 注意事项：
 * =========
 * - 支持 `throttle_ms` 作为节流手段（发送后延时），用于避免某些输出占满 CPU/带宽。
 */

#pragma once

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_uart_port.h"
#include "../communication_topic/topic_notify.hpp"

#include "../communication_topic/uart_topics.hpp"

namespace orb {
struct UartTxFrame;
}  // namespace orb

template<typename T, int DEPTH>
class RingSub;

// 统一 UART 发送任务：
// - 订阅 orb::uart_tx（通用字节流）
// - 唯一允许调用 bsp_uart_send 的模块

class UartTxTask {
public:
    bool Start();

    void Init() { (void)Start(); }

private:
    static void TaskEntry(void* arg);
    void Task();

    static BspUartId ToBspId(orb::UartPort p);

    bool started_ = false;

    osThreadId_t thread_ = nullptr;

    osEventFlagsId_t evt_ = nullptr;
    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};

    static constexpr uint32_t kEvtBit = 1u << 0;

    Notifier notifier_{nullptr, 0};

    StaticTask_t tcb_{};
    StackType_t stack_[512]{};
};
