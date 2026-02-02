# Drivers 模块

> 目标：提供与 MCU 外设/底层驱动相关的“薄封装”与 RTOS 任务，向上层暴露稳定、可复用的接口。
>
> 该模块尤其承担两类职责：
> 1) BSP/外设工具：GPIO/PWM/SPI/DWT/日志等。
> 2) **统一发送出口**：CAN/UART 的 TxTask（唯一允许调用底层 send）。

## 1. 模块职责与边界

**负责：**
- `bsp_*`：对底层外设能力进行封装（依赖 Board/Platform 的端口定义）。
- `can_tx_task`：订阅 Topic 并统一调用 `bsp_can_send()`。
- `uart_tx_task`：订阅 Topic 并统一调用 `bsp_uart_send()`。
- 以 daemon_supervisor 的方式对发送任务做“尽力而为”的在线监控。

**不负责：**
- 业务协议/帧格式拼装（应由上层模块产生待发帧并发布到 Topic）。
- 接收侧协议解析（通常在 Device/Communication/Interaction 等模块）。

## 2. 关键文件

- `can_tx_task.h/.cpp`
  - 统一 CAN 发送任务：订阅 `orb::can_tx`（通用 CAN 帧 Topic），并转为 `BspCanFrame` 调用 `bsp_can_send()`。
  - **原则：全仓唯一允许直接调用 `bsp_can_send()` 的位置**。
- `uart_tx_task.h/.cpp`
  - 统一 UART 发送任务：订阅 `orb::uart_tx`（通用字节流 Topic），并调用 `bsp_uart_send()`。
  - **原则：全仓唯一允许直接调用 `bsp_uart_send()` 的位置**。
- `bsp_log.h/.cpp`
  - 基于 SEGGER RTT 的日志输出（注意：默认不支持浮点格式化）。

## 3. 线程模型与数据流

### 3.1 CAN TxTask

- 任务名：`can_tx`
- 唤醒方式：`communication_topic` 的 `Notifier` 触发 `osEventFlags`。
- 数据流：
  - In: `orb::can_tx`（环形队列 Topic，元素为 `orb::CanTxFrame`）
  - Out: `bsp_can_send()` → CAN 外设
- 守护策略：
  - TxTask 不“接收外部数据”，因此采用 best-effort：仅在“实际发送”时 `feed()`，避免空转掩盖故障。

### 3.2 UART TxTask

- 任务名：`uart_tx`
- 唤醒方式：同上（EventFlags + Notifier）。
- 数据流：
  - In: `orb::uart_tx`（环形队列 Topic，元素为 `orb::UartTxFrame`）
  - Out: `bsp_uart_send()` → UART 外设
- 额外注意：
  - 支持 `throttle_ms`，用于对某些输出做节流（避免占满带宽/CPU）。

## 4. 常见误用与注意事项

- 禁止在业务层直接调用 `bsp_can_send()`/`bsp_uart_send()`：应发布到 `orb::can_tx`/`orb::uart_tx`，由 TxTask 统一发送。
- 若出现“队列堆积/发送不及时”，优先检查：
  - Topic 的发布频率与数据量是否合理；
  - TxTask 的优先级/节流延迟是否合适；
  - 底层 BSP 发送是否被阻塞。
