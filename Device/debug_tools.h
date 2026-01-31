#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include <cstdint>
#include "bsp_uart_port.h"

class DebugTools
{

private:
    BspUartHandle uart_ = nullptr;

public:
    void VofaInit();
    void VofaSendFloat(float data);
    void VofaSendTail();
    void VofaReceiveCallback(uint8_t *buffer, uint16_t length);
};

#endif // DEBUG_TOOLS_H_
