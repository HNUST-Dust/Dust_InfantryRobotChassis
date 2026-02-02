# Dust_InfantryRobotChassis

本仓库为 Dust 步兵底盘工程（STM32H723 + FreeRTOS/CMSIS-OS2）。工程核心约束包括：
- 通信以 Topic 为唯一出口（uORB 风格 pub-sub）。
- CAN/UART 发送必须经由唯一 TxTask（Drivers）落到 BSP。
- daemon_supervisor 以“收到新外部数据/实际 TX”作为在线判据（避免任务空转误判）。

## 文档

- 仓库文档导航（建议入口）：[docs/README.md](docs/README.md)
