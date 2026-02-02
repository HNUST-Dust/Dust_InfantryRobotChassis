/**
 * @file can_tx_task.h
 * @brief 统一 CAN 发送任务（Topic -> BSP CAN）
 *
 * 设计思路：
 * =========
 * - 通过 `communication_topic` 的 Notifier/EventFlags 机制，在有新待发帧时唤醒任务。
 * - 订阅 `orb::can_tx`（通用 CAN 帧 Topic），并将其转为 `BspCanFrame` 调用 `bsp_can_send()`。
 * - 该任务是全仓 CAN 发送的唯一出口：业务侧只“发布待发帧”，不直接碰 BSP。
 *
 * 线程模型：
 * =========
 * - 单任务循环 drain ring topic；不保证多任务并发安全（但外部只应通过 Topic 交互）。
 *
 * 守护策略：
 * =========
 * - TxTask 本身不接收外部数据；采用 best-effort 在线监控：仅在“实际发送”时 feed，避免空转掩盖卡死。
 */

#pragma once

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can_port.h"
#include "../communication_topic/topic_notify.hpp"

namespace orb {
struct CanTxFrame;
}  // namespace orb

template<typename T, int DEPTH>
class RingSub;

// 统一 CAN 发送任务：
// - 订阅 orb::can_tx（通用帧）
// - 唯一允许调用 bsp_can_send 的模块

class CanTxTask {
public:
    bool Start();

    void Init() { (void)Start(); }

private:
    static void TaskEntry(void* arg);
    void Task();

    bool started_ = false;

    BspCanHandle can1_ = nullptr;
    BspCanHandle can2_ = nullptr;
    BspCanHandle can3_ = nullptr;

    osThreadId_t thread_ = nullptr;

    osEventFlagsId_t evt_ = nullptr;
    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};

    static constexpr uint32_t kEvtBit = 1u << 0;

    Notifier notifier_{nullptr, 0};

    StaticTask_t tcb_{};
    StackType_t stack_[512]{};
};
