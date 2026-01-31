#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Standalone module/service singletons (decoupled from Robot)
//
// Purpose:
// - Ensure wiring callbacks (CAN/UART ISR adapters) and bring-up initialization
//   operate on the SAME module instances.
// - Avoid accessing internals of Robot (Robot is a pure "fusion & publish" node).

void Modules_Init(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class DebugTools;
class Chassis;
class Gimbal;
class McuComm;
class Supercap;
class Referee;
class MotorActuatorTask;

DebugTools& Modules_DebugTools();
Chassis&    Modules_Chassis();
Gimbal&     Modules_Gimbal();
McuComm&    Modules_McuComm();
Supercap&   Modules_Supercap();
Referee&    Modules_Referee();
MotorActuatorTask& Modules_MotorActuator();

#endif
