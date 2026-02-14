# Communication 模块说明

## 1. 概述

`Communication/` 负责与外部 MCU/上位机的通信解包、状态发布，以及与 Topic 系统的对接。

当前目录内的核心实现为 `McuComm`：
- 从 CAN 接收外部数据（遥控/自瞄/IMU 信息等）。
- 将解析后的数据发布到 Topic，供其它模块订阅。
- 对外发送采用“Topic → 统一发送出口”的策略：通过订阅待发送 Topic，在专用 TxTask 中组帧并发布到 `orb::can_tx`。

## 2. 关键文件

- `mcu_comm.h/.cpp`
  - `class McuComm`：CAN 绑定、接收回调、数据处理、发布 Topic、自动发送 TxTask。

## 3. 线程模型

`McuComm` 涉及两类执行上下文：

1) RX 回调上下文
- `CanRemoteControlRxCpltCallback` / `CanAutoAimInfoRxCpltCallback` / `CanImuInfoRxCpltCallback`
- 由平台层 CAN 接收回调触发。
- 约束：必须快速执行；不阻塞；做最小解析并发布 Topic。

2) TxTask 线程上下文（CMSIS-RTOS2 线程）
- `StartAutoTx()` 创建事件标志与线程。
- `TxTask()` 等待 Topic notifier 唤醒，批量读取 RingTopic 并发布到 `orb::can_tx`。

## 4. 数据流

- In（来自外部 MCU）：
  - CAN2：按 ID 分发
    - `REMOTE_CONTROL_ID` → `McuControl` Topic
    - `AUTOAIM_INFO_ID` → `McuAutoAim` Topic
    - `IMU_INFO_ID` → `McuImu` Topic

- Out（发往外部 MCU）：
  - 订阅 `orb::gimbal_info_tx`（RingTopic）
  - 组帧后发布到 `orb::can_tx`（由统一 CAN TxTask 负责最终发送）

## 5. 守护/容错

- `McuComm::Start()` 会向 `daemon_supervisor` 注册一个 `DaemonClient`。
- feed 判据：**只有在 RX 回调收到新外部数据**时才 feed。
  - 目的：避免“无流量但任务空转”导致误判在线。

## 6. 注意事项

- 不要在业务模块中直接调用 BSP CAN 发送；应发布 `orb::can_tx`。
- `McuComm::Bind()` 与 `McuComm::Start()` 语义分离：先绑定硬件句柄/ID，再启动任务与守护。
