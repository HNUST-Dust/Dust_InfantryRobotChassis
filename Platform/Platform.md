# Platform 模块

> 目标：提供“面向上层的 HAL-free 端口抽象”，将底层 MCU 外设实现（Board/Drivers）与上层业务模块解耦。

## 1. 模块职责与边界

**负责：**
- 提供上层可用的 CAN/UART 抽象接口（handle + send + RX callback 注册）。
- 通过 C ABI 形式导出接口，供 C/C++ 模块统一调用。

**不负责：**
- 具体外设初始化细节（引脚/时钟/DMA/IRQ）：这些由 `Board/dm-h723` 的 CubeMX 生成代码与 BSP 实现承担。
- 上层发送策略：工程要求 CAN/UART 发送必须经由 `Drivers/CanTxTask` / `Drivers/UartTxTask` 的“唯一出口”。

## 2. 关键文件

- `bsp_can_port.h`
  - CAN 端口抽象：`bsp_can_get/init/add_rx_callback/send`。
  - `BspCanFrame` 统一携带 STD/EXT、DATA/REMOTE、CAN-FD/BRS 等标志。

- `bsp_uart_port.h`
  - UART 端口抽象：`bsp_uart_get/init/send`。

## 3. 线程模型与约束

- **发送路径（TX）：**
  - 上层模块不应直接调用 `bsp_can_send()` / `bsp_uart_send()`。
  - 正确做法：发布到 `communication_topic`（如 `orb::can_tx` / `orb::uart_tx`），由 TxTask 统一 drain 并调用 BSP send。

- **接收路径（RX callback）：**
  - 回调通常在中断或 ISR 相关上下文触发（取决于具体实现）。
  - 回调必须保持轻量：不阻塞、不做大量计算、不申请动态内存；推荐只做解析/复制并发布 Topic，交由任务处理。

- **回调注册：**
  - CAN 提供两种接口：
    - `bsp_can_init()`：单回调（覆盖式，legacy）。
    - `bsp_can_add_rx_callback()`：追加式，多回调按注册顺序调用（内部列表满则返回 false）。

## 4. 常见误用

- 在业务模块中直接调用 `bsp_*_send()`：会绕过 TxTask，破坏“唯一出口”约束，也可能导致守护判据失真。
- 在 RX callback 中执行阻塞操作或复杂逻辑：可能导致 ISR 抢占过久、丢包、系统抖动。
