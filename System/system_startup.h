#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// L2: Board bring-up (board-local HW on the PCB, e.g. LED, onboard IMU power/reset, etc.)
void Board_BringUp(void);

// L3: BSP bring-up (start IO services: UART/CAN DMA+IRQ, etc.)
void Bsp_BringUp(void);

// L4: External modules bring-up (motors, supercap, referee, etc.)
void Modules_BringUp(void);

// L5: App bring-up
void App_Start(void);

// One-shot entry to run all bring-up stages in order (called from RTOS init task)
void System_Boot(void);

#ifdef __cplusplus
}
#endif
