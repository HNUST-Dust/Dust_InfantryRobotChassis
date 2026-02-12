# Interaction 模块说明

## 1. 概述

`Interaction/` 负责“应用层的装配与入口”，把平台层（CAN/UART）收到的数据分发给各业务模块，并提供系统启动入口。

职责边界：
- 负责：
  - 系统启动入口 `Init()` → 调用系统启动/初始化（当前为 `System_Boot()`）。
  - 平台 IO 绑定：注册 UART/CAN 回调，将帧转发到对应业务模块实例。
- 不负责：
  - 不直接发送 CAN/UART（发送应由各自 TxTask/Topic 管道完成）。
  - 不做业务控制算法（云台/底盘控制逻辑在 app_* 或 Device/ 模块）。

## 2. 关键文件

- `Init.h/.cpp`
  - CubeMX/RTOS 生成代码的 legacy 入口 `Init(void)`。
  - 当前实现：`Init()` 调用 `System_Boot()`。
- `AppWiring.h/.cpp`
  - 平台 IO 回调“装配表”。
  - 负责把 BSP 回调扇出到各模块：
    - UART7 → DebugTools 的 VOFA 接收
    - UART1 → 裁判系统 Referee 接收
    - CAN1/2/3 → 电机、上位机/MCU、超级电容等模块
- `app_chassis.*` / `app_gimbal.*`
  - 应用层（底盘/云台）逻辑入口，通常以 `*_Instance()` 形式提供单例访问。

## 3. 线程模型

`Interaction/` 本身不创建 RTOS 任务：
- `Init()` 在系统启动阶段被调用。
- `App_WirePlatformIo()` 只做回调注册；回调在 BSP 对应的接收上下文被触发（可能是 IRQ 或专用接收任务，取决于 BSP 实现）。

因此回调函数中应遵循：
- 只做“快速转发/拷贝/投递通知”，不要做重计算。
- 避免阻塞。

## 4. 数据流（以现有实现为准）

- UART7 → `DebugTools_Instance().VofaReceiveCallback(buffer, length)`
- UART1 → `Referee_Instance().RxCpltCallback(buffer, length)`
- CAN1/2/3 → 按 ID 分发到各电机实例 `CanRxCpltCallback(frame)`
- CAN2（部分 ID）→ `McuComm_Instance().Can*RxCpltCallback(frame)`
- CAN3（0x100）→ `Supercap_Instance().CanRxCpltCallback(frame)`

## 5. 守护/容错

`Interaction/` 自身不注册 daemon client。
但是它转发的回调可能会间接触发某些模块在“收到新外部数据”时 feed（例如 McuComm 的 RX 回调）。
