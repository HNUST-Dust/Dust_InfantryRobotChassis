#include "AppContext.h"

#include "Robot.h"

// Keep global objects in one TU to avoid scattered globals.
static Robot s_robot;

Robot& App_Robot(void)
{
    return s_robot;
}

void App_Init(void)
{
    // App-layer bring-up only.
    // IO wiring is handled by Bsp_BringUp() in System_Boot().
    s_robot.Start();
}
