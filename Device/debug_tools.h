/**
 * @file debug_tools.h
 * @brief 调试工具设备封装（VOFA）：通过 Topic 化 UART TX 发送调试数据，并以 RX 作为在线判据。
 *
 * **定位**
 * - 该模块用于与 PC 侧调试工具（VOFA 等）交互。
 * - 发送路径遵循“唯一出口”原则：只发布 `orb::uart_tx`，由 Drivers/UartTxTask 统一发送。
 *
 * **在线判据/守护**
 * - 仅在收到新的 VOFA RX 帧时 feed（由 `VofaReceiveCallback()` 驱动）。
 * - Start() 会写入 baseline 时间戳，避免刚启动就被判离线。
 *
 * **约束**
 * - 仅保存 UART 句柄作为 RX wiring 的引用；实际发送使用绑定的 `orb::UartPort`。
 */

#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include <cstdint>
#include "bsp_uart_port.h"

#include "../communication_topic/uart_topics.hpp"

class DebugTools
{

private:
    BspUartHandle uart_ = nullptr;
    orb::UartPort vofa_port_ = orb::UartPort::U7;
    bool started_ = false;

public:
    // Singleton accessor (explicit call-site; no global free-function)
    static DebugTools& Instance();

    void Init(BspUartHandle uart, orb::UartPort vofa_port);

    void VofaSendFloat(float data);
    void VofaSendTail();
    void VofaReceiveCallback(uint8_t *buffer, uint16_t length);
};

#endif // DEBUG_TOOLS_H_
