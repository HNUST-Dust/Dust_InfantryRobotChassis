# Device 模块说明

## 1. 概述

`Device/` 聚合“具体设备/外设相关”的实现：IMU、遥控、裁判系统、超级电容、电机执行器与控制相关 glue 等。

职责边界：
- 负责：
  - 传感器/外设的驱动与中间层封装（例如 BMI088、裁判系统、超级电容等）。
  - 与平台层（CAN/UART/PWM/DWT）交互的设备逻辑。
  - 部分实时任务（例如 INS 更新、执行器任务）与其数据接口。
- 不负责：
  - 平台 BSP 的底层实现（在 Drivers/Board/Platform）。
  - 通用算法库（在 Algorithm/）。

## 2. 关键文件（示例）

- `ins_task.h/.cpp`
  - IMU/INS 数据更新与姿态解算相关 glue。
  - 依赖 `Algorithm/estimation/QuaternionEKF`。
  - 使用 DWT 获取 dt，周期性读取 BMI088。
- 电机驱动（DJI/DM）
  - 电机实例在对应业务模块中创建与绑定（如 `Interaction/app_chassis.*`、`Interaction/app_gimbal.*`）。
  - CAN TX 仍通过 `orb::can_tx` 由 TxTask 统一发送。
- `controller.*`
  - 控制器相关的整合层（与算法/设备数据交互）。
- `debug_tools.*`
  - 调试/VOFA 等工具链入口。

## 3. 线程模型（以现有实现为准）

- 部分文件提供 `*_Task()` 形式的周期函数，可能由 RTOS 任务调用。
- 部分设备通过回调接收（例如 UART/CAN RX），这些回调通常由 `Interaction/AppWiring` 绑定。

建议：
- ISR/回调中只做轻量处理，复杂计算放到任务中。

## 4. 守护/容错

- 设备模块如果依赖“外部输入数据”才能认为在线（例如外部 MCU、裁判系统数据等），推荐在**收到新数据时** feed daemon client。
- 不建议按固定周期无条件 feed（会掩盖通信中断/传感器停更）。
