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
    bool bound_ = false;
    bool started_ = false;

public:
    // Bind UART handle (for RX wiring reference) and select VOFA TX port (for orb::uart_tx)
    void Bind(BspUartHandle uart, orb::UartPort vofa_port);

    bool Start();

    void VofaInit();
    void VofaSendFloat(float data);
    void VofaSendTail();
    void VofaReceiveCallback(uint8_t *buffer, uint16_t length);
};

// Module singleton accessor (no Context/service-locator layer)
DebugTools& DebugTools_Instance();

#endif // DEBUG_TOOLS_H_
