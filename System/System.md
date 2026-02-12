# System 模块

> 目标：提供工程的“统一启动序列”（staged bring-up），将 CubeMX/RTOS 生成代码与项目自身的模块化初始化解耦。

## 1. 模块职责与边界

**负责：**
- 定义并执行一次性的分层启动流程：BSP/板级/模块/应用。
- 约定各层初始化的先后顺序与依赖（例如 DWT 时间基、daemon_supervisor 必须早于各模块注册）。

**不负责：**
- 任何具体外设驱动细节（由 `Platform/`、`Drivers/`、`Board/` 实现）。
- 业务策略与控制逻辑（由 `Interaction/`、`Device/` 等模块实现）。

## 2. 关键文件

- `system_startup.h/.cpp`
  - 定义 staged bring-up 的分层函数与一站式入口 `System_Boot()`。
  - 将“哪些模块在系统启动时拉起”集中在一处，便于审查启动顺序与依赖。

## 3. 启动时序（当前实现）

`System_Boot()` 的执行顺序如下：

1) **L3: BSP bring-up**：`Bsp_BringUp()`
- 作用：尽早启动 IO 服务（UART/CAN 的 DMA/IRQ、idle 等）。
- 当前实现：调用 `App_WirePlatformIo()`（见 `Interaction/AppWiring.*`）。

2) **L2: Board bring-up**：`Board_BringUp()`
- 作用：板级“PCB 上的硬件”初始化（例如 LED/IMU 供电复位等）。
- 当前实现：初始化 DWT 时间基 `dwt_init(480)`，供 daemon_task/控制环路使用 `dwt_get_timeline_ms()`。

3) **L4: Modules bring-up**：`Modules_BringUp()`
- 作用：启动系统级模块任务与守护。
- 当前实现包含：
  - `DaemonSupervisor::init()` + `set_system_fault_hook()`，并启动 daemon 任务（代码注释标记为 100Hz）。
  - 启动统一发送任务：`CanTxTask` / `UartTxTask`（系统唯一允许调用 `bsp_can_send/bsp_uart_send` 的位置）。
  - 绑定电机实例：在 `Chassis/Gimbal` 模块内创建并绑定 CAN（每电机独立实例）。
  - 启动业务模块：底盘 `Chassis_Instance().Start()`、云台 `Gimbal_Instance().Start()`。
  - 初始化并启动外设模块：上位机/下板通信 `McuComm`（CAN2）、超级电容 `Supercap`（CAN3）、裁判系统 `Referee`（UART1）、调试工具 `DebugTools`（UART7）。

4) **L5: App bring-up**：`App_Start()`
- 当前为分布式架构：业务模块已在 Modules_BringUp() 启动，因此这里为空实现（只保留层级语义）。

## 4. 入口与生成代码对接

工程存在两条兼容入口（均最终调用 `System_Boot()`）：
- CubeMX 生成的 FreeRTOS `defaultTask`：在 `Board/dm-h723/Src/freertos.c` 的 `StartDefaultTask()` 中调用。
- 兼容入口 `Interaction/Init()`：`Interaction/Init.cpp` 中直接调用 `System_Boot()`，用于“生成代码世界”与工程启动序列解耦。

实际使用哪条入口取决于当前工程如何集成生成代码；无论入口来源，`System_Boot()` 的分层启动顺序应保持一致。

## 5. 线程模型与注意事项

- `System_Boot()` 设计为一次性调用（one-shot）：在 RTOS 初始化任务中运行一次后退出（见 `StartDefaultTask()` 的 `osThreadExit()`）。
- `Modules_BringUp()` 内部会创建多个任务/组件；后续不应再次调用 `System_Boot()`，避免重复创建任务或重复注册 daemon client。
- `daemon_system_fault()` 是 best-effort 安全降级：当前会请求底盘/云台退出，并通过 Topic 发布 DM 电机管理指令（`orb::gimbal_dm_admin_cmd`）。
