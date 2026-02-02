# communication_topic 模块说明（Topic / Pub-Sub）

## 1. 概述

`communication_topic/` 提供一个轻量级的发布-订阅（Pub-Sub）消息通信框架，风格参考 PX4 uORB。

核心价值：
- 解耦：发布者与订阅者不直接依赖彼此。
- 低开销：静态内存为主、无动态分配的使用方式。
- 事件驱动：通过 `Notifier` + RTOS EventFlags 实现“发布时唤醒订阅者”。

## 2. 核心概念

- `Topic<T>`：单缓冲主题，只保留最新一条数据（适用于“只关心最新状态”）。
- `RingTopic<T, DEPTH>`：环形缓冲主题，保存最近 DEPTH 条数据（适用于“不能丢数据”的流式场景）。
- `Publisher<T>`：发布者薄封装，语义更清晰。
- `Subscription<T>` / `Sub<T>`：订阅者，跟踪 generation 判断是否有更新。
- `RingSub<T, DEPTH>`：环形订阅者，从 `RingTopic` 顺序消费。
- `Notifier`：将 `Topic`/`RingTopic` 的 publish 事件映射到 RTOS EventFlags。
- `TopicWaiter`：任务侧封装（创建 EventFlags、持有 Notifier、提供 wait）。

## 3. 线程模型与约束

- 单写多读（Single Writer, Multi Reader）是默认安全模型。
  - 同一个 `Topic<T>` 建议只由一个发布者发布。
  - 多个订阅者可以并行读取。

- 回调/ISR 场景：
  - 如果在 ISR 发布 Topic，需要确保 `notify()` 使用的 RTOS API 在 ISR 下安全。
  - 当前实现基于 CMSIS-RTOS2 EventFlags（底层通常映射到 FreeRTOS EventGroup）。

## 4. 事件驱动典型用法

1) 任务初始化：创建 `TopicWaiter`，把 waiter 的 `notifier()` 注册到 Topic/RingTopic。
2) 任务循环：`wait()` 阻塞等待发布事件，然后用 `Sub/RingSub` 批量 `copy()` 取走数据。
3) 发布者：在发布新数据时自动触发 `notifier->notify()`。

## 5. 关键文件

- `topic.hpp`：Topic/RingTopic/MultiTopic/Subscription/RingSub 的核心实现与内存序说明。
- `topic_pubsub.hpp`：`Publisher`/`Sub` 语义包装。
- `topic_notify.hpp`：`Notifier` 实现。
- `topic_wait.hpp`：`TopicWaiter` 工具。
- `*_topics.hpp`：工程内具体 Topic 的声明（CAN/UART/MCU/设备等）。
