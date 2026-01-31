#include "ModulesContext.h"

#include "../Device/debug_tools.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "../Communication/dvc_MCU_comm.h"
#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Device/motor_actuator_task.h"

// Keep module singletons in one TU to avoid duplicated instances across files.
static DebugTools s_debug_tools;
static Chassis s_chassis;
static Gimbal s_gimbal;
static McuComm s_mcu_comm;
static Supercap s_supercap;
static Referee s_referee;
static MotorActuatorTask s_motor_actuator;

void Modules_Init(void)
{
    // Intentionally empty.
    // Modules are initialized in System/Modules_BringUp().
    // This hook exists in case we later need lazy init or dependency ordering.

    // Bind CAN handles for motor actuator task
    s_motor_actuator.Bind(bsp_can_get(BSP_CAN_BUS1), bsp_can_get(BSP_CAN_BUS2), bsp_can_get(BSP_CAN_BUS3));
}

DebugTools& Modules_DebugTools() { return s_debug_tools; }
Chassis&    Modules_Chassis()    { return s_chassis; }
Gimbal&     Modules_Gimbal()     { return s_gimbal; }
McuComm&    Modules_McuComm()    { return s_mcu_comm; }
Supercap&   Modules_Supercap()   { return s_supercap; }
Referee&    Modules_Referee()    { return s_referee; }
MotorActuatorTask& Modules_MotorActuator() { return s_motor_actuator; }
