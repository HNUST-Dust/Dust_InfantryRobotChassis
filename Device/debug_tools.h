#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include "stm32h7xx_hal.h"

class DebugTools
{

private:

public:
    void VofaInit();
    void VofaSendFloat(float data);
    void VofaSendTail();
    void VofaReceiveCallback(uint8_t *buffer, uint16_t length);
};

#endif // DEBUG_TOOLS_H_
