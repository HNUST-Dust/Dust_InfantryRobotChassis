# 仓库文档导航（Docs Index）

这份索引把仓库里的关键模块文档串起来，便于快速理解“启动链路 / Topic 通信 / 唯一发送出口 / 守护策略”等工程约束。

## 1) 从这里开始

- 统一标准（建议先读）：[docs/DocumentationStandard.md](DocumentationStandard.md)
- 工程启动与分层 bring-up：[System/System.md](../System/System.md)

## 2) 架构与基础设施

- Topic / Pub-Sub 框架：[communication_topic/CommunicationTopic.md](../communication_topic/CommunicationTopic.md)
- Platform 端口抽象（CAN/UART，HAL-free）：[Platform/Platform.md](../Platform/Platform.md)
- Drivers（含 CAN/UART TxTask 唯一发送出口）：[Drivers/Drivers.md](../Drivers/Drivers.md)
- daemon_supervisor（健康监控/离线回调/系统 fault hook）：[daemon_supervisor/DaemonSupervisor.md](../daemon_supervisor/DaemonSupervisor.md)

## 3) 业务与设备模块

- Interaction（启动 glue + UART/CAN RX 装配分发）：[Interaction/Interaction.md](../Interaction/Interaction.md)
- Communication（外部 MCU 通信解包/发布 Topic）：[Communication/Communication.md](../Communication/Communication.md)
- Device（执行器/裁判/超级电容/调试工具等设备封装）：[Device/Device.md](../Device/Device.md)
- Algorithm（算法库分层与使用建议）：[Algorithm/Algorithm.md](../Algorithm/Algorithm.md)

## 4) 板级与生成代码

- Board/dm-h723（CubeMX + 启动入口 + OpenOCD 配置）：[Board/dm-h723/Board.md](../Board/dm-h723/Board.md)

## 5) 推荐阅读路径

- 新同学快速上手（先理解“链路与约束”）：
  1. [System/System.md](../System/System.md)
  2. [communication_topic/CommunicationTopic.md](../communication_topic/CommunicationTopic.md)
  3. [Drivers/Drivers.md](../Drivers/Drivers.md)
  4. [Platform/Platform.md](../Platform/Platform.md)
  5. [Interaction/Interaction.md](../Interaction/Interaction.md)
  6. [Device/Device.md](../Device/Device.md)
  7. [Communication/Communication.md](../Communication/Communication.md)
  8. [Algorithm/Algorithm.md](../Algorithm/Algorithm.md)
  9. [daemon_supervisor/DaemonSupervisor.md](../daemon_supervisor/DaemonSupervisor.md)

## 6) 模块关系速览（当前实现口径）

```
Board (CubeMX)
  -> RTOS defaultTask
  -> System_Boot()  (System)
      -> App_WirePlatformIo() (Interaction)   [RX wiring]
      -> DaemonSupervisor init (daemon_supervisor)
      -> CanTxTask/UartTxTask (Drivers)       [TX唯一出口]
      -> McuComm/Referee/Supercap/... (Communication/Device)
      -> Chassis/Gimbal (Interaction)

业务/设备模块发送：publish orb::can_tx / orb::uart_tx
  -> Drivers/*TxTask drain
  -> bsp_can_send / bsp_uart_send
```
