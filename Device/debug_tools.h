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
};

#endif // DEBUG_TOOLS_H_
