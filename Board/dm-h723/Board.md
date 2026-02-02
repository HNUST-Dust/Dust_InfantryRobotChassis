# Board/dm-h723（STM32H723 板级支持）

> 目标：承载与具体 PCB/MCU 绑定的板级工程（CubeMX 生成代码 + 链接脚本 + OpenOCD 配置），为上层模块提供可运行的底座。

## 1. 模块职责与边界

**负责：**
- MCU 启动与系统初始化（HAL_Init、时钟树、外设初始化、IRQ 入口）。
- FreeRTOS/CMSIS-RTOS2 的基础启动与 defaultTask 创建。
- 链接脚本与内存布局（FLASH/RAM 分区、向量表等）。
- 调试/烧录配置（OpenOCD、CMSIS-DAP）。

**不负责：**
- 业务模块的启动顺序与模块依赖管理：这由 `System/System_Boot()` 统一决定。
- “唯一发送出口”等架构约束：这些由 `Drivers/*TxTask` + `communication_topic` 实现。

## 2. 关键文件与目录

- `Dust_Chassis.ioc`
  - CubeMX 工程文件，外设、Pin、时钟、RTOS 配置的来源。

- `Src/main.c`
  - HAL + 外设初始化入口：`HAL_Init()` -> `SystemClock_Config()` -> `MX_*_Init()` -> `osKernelInitialize()` -> `MX_FREERTOS_Init()` -> `osKernelStart()`。

- `Src/freertos.c`
  - defaultTask 创建与 `StartDefaultTask()` 实现。
  - 当前 `StartDefaultTask()` 中调用 `System_Boot()` 并 `osThreadExit()`：将“生成代码世界”的启动入口交给 System 模块。

- `Src/stm32h7xx_it.c` / `Inc/stm32h7xx_it.h`
  - 中断向量对应的 ISR 框架与 HAL 回调入口（DMA/UART/FDCAN/TIM/USB 等）。

- `STM32H723VGTX_FLASH.ld` / `STM32H723VGTX_RAM.ld`
  - 链接脚本，定义内存区域与段布局。

- `config/openocd_dap.cfg`
  - OpenOCD 配置（CMSIS-DAP + SWD + stm32h7x target）。
  - 对应 VS Code 任务“Flash STM32”会使用该 cfg 并烧录 `build/*.bin`。

## 3. 启动链路（从复位到业务任务）

典型路径：
1) 复位后进入启动文件（`startup_stm32h723vgtx.s`）
2) `main()`（`Src/main.c`）完成 HAL/时钟/外设初始化
3) 启动 RTOS：`osKernelInitialize()` + `MX_FREERTOS_Init()` + `osKernelStart()`
4) defaultTask 运行：`StartDefaultTask()`（`Src/freertos.c`）
5) 调用 `System_Boot()`（见 `System/system_startup.cpp`）
   - BSP bring-up（IO 服务）
   - Board bring-up（DWT 时间基等）
   - Modules bring-up（daemon、TxTask、执行器层、底盘/云台/外设模块）

## 4. 开发与维护建议（针对生成代码）

- CubeMX 重新生成代码时，只应在 `/* USER CODE BEGIN ... */` 与 `/* USER CODE END ... */` 区间写自定义内容。
- 对于需要长期维护的“工程语义说明”，优先写在模块文档（本文件）或 USER CODE Header 区，避免被覆盖。

## 5. 烧录与调试

- OpenOCD 配置位于 `config/openocd_dap.cfg`，默认使用 CMSIS-DAP + SWD。
- VS Code 任务：`Flash STM32`
  - 使用 OpenOCD 将 `build/Dust_InfantryRobotChassis.bin` 写入 `0x08000000`。
