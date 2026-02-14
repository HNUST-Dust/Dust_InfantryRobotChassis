/**
 * @file debug_tools.cpp
 * @brief DebugTools 实现：VOFA 数据打包并发布到 `orb::uart_tx`，以及 RX 驱动的在线 feed。
 */

#include "debug_tools.h"

#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "../communication_topic/uart_topics.hpp"

DebugTools& DebugTools::Instance()
{
    static DebugTools inst;
    return inst;
}

void DebugTools::Init(BspUartHandle uart, orb::UartPort vofa_port)
{
    if (started_) {
        configASSERT(false);
        return;
    }
    configASSERT(uart != nullptr);
    if (uart == nullptr) {
        return;
    }

    uart_ = uart;
    vofa_port_ = vofa_port;

    started_ = true;
}

void DebugTools::VofaSendFloat(float data)
{
    orb::UartTxFrame pkt{};
    pkt.port = vofa_port_;
    pkt.len = 4;
    static_assert(sizeof(float) == 4, "VOFA float must be 4 bytes");
    std::memcpy(pkt.bytes, &data, 4);
    orb::uart_tx.publish(pkt);
}

void DebugTools::VofaSendTail()
{
    orb::UartTxFrame pkt{};
    pkt.port = vofa_port_;
    pkt.len = 4;
    pkt.bytes[0] = 0x00;
    pkt.bytes[1] = 0x00;
    pkt.bytes[2] = 0x80;
    pkt.bytes[3] = 0x7f;
    orb::uart_tx.publish(pkt);
}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
}