/**
 * @file system_startup.h
 * @brief 工程统一启动序列（staged bring-up）对外声明
 *
 * 设计思路：
 * =========
 * - 将系统启动拆成多个“层级”（Board/BSP/Modules/App），明确依赖并固定顺序。
 * - 统一入口为 `System_Boot()`：通常在 RTOS 初始化任务（defaultTask/Init glue）中 one-shot 调用。
 *
 * 层级约定（从底到上）：
 * ====================
 * - L2 Board：板级硬件（PCB 上资源、时间基等）。
 * - L3 BSP：IO 服务就绪（UART/CAN 的 DMA/IRQ/idle 等）。
 * - L4 Modules：启动系统模块任务（TxTask、执行器层、守护等）。
 * - L5 App：业务层启动（当前为分布式架构，可能为空实现）。
 */

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

// entry to run all bring-up stages in order
void startup_thread(void *argument);

#ifdef __cplusplus
}
#endif
