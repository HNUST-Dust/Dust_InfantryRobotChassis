/**
 * @file uart_tx_task.cpp
 * @brief `Drivers/uart_tx_task.h` 的实现（drain `orb::uart_tx` 并调用 `bsp_uart_send()`）
 *
 * 实现要点：
 * =========
 * - 使用 `RingSub<orb::UartTxFrame, 32>` drain 待发字节流。
 * - 通过 `orb::UartPort -> BspUartId` 映射选择底层句柄。
 * - 支持 `throttle_ms`：对每包发送后延时（尽量减少对实时路径的干扰）。
 */

#include "uart_tx_task.h"

#include <cstring>

#include "../communication_topic/uart_topics.hpp"

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

DaemonClient* s_uart_tx_daemon = nullptr;
} // namespace

BspUartId UartTxTask::ToBspId(orb::UartPort p)
{
    switch (p) {
        case orb::UartPort::U1: return BSP_UART1;
        case orb::UartPort::U2: return BSP_UART2;
        case orb::UartPort::U3: return BSP_UART3;
        case orb::UartPort::U4: return BSP_UART4;
        case orb::UartPort::U5: return BSP_UART5;
        case orb::UartPort::U6: return BSP_UART6;
        case orb::UartPort::U7: return BSP_UART7;
        case orb::UartPort::U8: return BSP_UART8;
        case orb::UartPort::U10: return BSP_USART10;
        default: return BSP_UART1;
    }
}

bool UartTxTask::Start()
{
    if (started_) {
        configASSERT(false);
        return false;
    }

    // Tx tasks don't receive external data; monitor them in a best-effort way.
    // Feed is driven by actual TX activity.
    {
        static DaemonClient s_daemon(
            3000,
            nullptr,
            this,
            DaemonClient::Domain::COMM,
            DaemonClient::FaultLevel::WARN,
            DaemonClient::Priority::HIGH);
        s_uart_tx_daemon = &s_daemon;
        (void)DaemonSupervisor::register_client(s_uart_tx_daemon);
        s_uart_tx_daemon->feed(now_ms());
    }

    started_ = true;

    evt_attr_ = osEventFlagsAttr_t{
        .name = "uart_tx_evt",
        .cb_mem = &evt_cb_,
        .cb_size = sizeof(evt_cb_),
    };
    evt_ = osEventFlagsNew(&evt_attr_);
    if (!evt_) {
        configASSERT(false);
        return false;
    }

    notifier_ = Notifier(evt_, kEvtBit);
    orb::uart_tx.register_notifier(&notifier_);

    const osThreadAttr_t attr{
        .name = "uart_tx",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t)osPriorityLow,
    };

    thread_ = osThreadNew(&UartTxTask::TaskEntry, this, &attr);
    if (!thread_) {
        configASSERT(false);
        return false;
    }
    return true;
}

void UartTxTask::TaskEntry(void* arg)
{
    static_cast<UartTxTask*>(arg)->Task();
}

void UartTxTask::Task()
{
    RingSub<orb::UartTxFrame, 32> sub(orb::uart_tx);

    for (;;) {
        (void)osEventFlagsWait(evt_, kEvtBit, osFlagsWaitAny, 1);

        auto send_bytes = [&](orb::UartPort port, const uint8_t* bytes, uint16_t len, uint16_t throttle_ms) {
            if (!bytes || len == 0) {
                return;
            }

            BspUartHandle h = bsp_uart_get(ToBspId(port));
            if (!h) {
                return;
            }

            if (s_uart_tx_daemon) {
                s_uart_tx_daemon->feed(now_ms());
            }

            (void)bsp_uart_send(h, const_cast<uint8_t*>(bytes), len);
            if (throttle_ms > 0) {
                osDelay(throttle_ms);
            }
        };

        // A) drain generic UART frames
        orb::UartTxFrame pkt{};
        while (sub.copy(pkt)) {
            if (pkt.len == 0 || pkt.len > sizeof(pkt.bytes)) {
                continue;
            }
            send_bytes(pkt.port, pkt.bytes, pkt.len, pkt.throttle_ms);
        }

        // 兜底：避免空转
        osDelay(1);
    }
}
