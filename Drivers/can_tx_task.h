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
