#pragma once

#include "topic.hpp"

// 通用 UART Tx Topic：所有模块只负责“描述要发的字节流”，由统一 UartTxTask 串行发送。
// 约束：该 Topic 是驱动层统一出口，上层模块不应直接调用 bsp_uart_send。

namespace orb {

enum class UartPort : uint8_t {
    U1 = 1,
    U2 = 2,
    U3 = 3,
    U4 = 4,
    U5 = 5,
    U6 = 6,
    U7 = 7,
    U8 = 8,
    U10 = 10,
};

struct UartTxFrame {
    UartPort port = UartPort::U1;

    uint16_t len = 0;
    uint8_t bytes[256] = {0};

    // 可选节流：用于类似裁判系统 UI 这类需要限频的通道。
    // 0 表示不延时。
    uint16_t throttle_ms = 0;
};

inline RingTopic<UartTxFrame, 32> uart_tx;

}  // namespace orb
