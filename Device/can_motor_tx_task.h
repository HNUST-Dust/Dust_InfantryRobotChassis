#pragma once

#include "cmsis_os2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can_port.h"
#include "../communication_topic/topic_notify.hpp"

namespace orb {
struct DjiCurrentGroupCmd;
}  // namespace orb

template<typename T, int DEPTH>
class RingSub;

template<typename T, int DEPTH>
class RingTopic;

// CAN 电机发送任务：订阅 Topic 队列，将语义化 cmd 打包成 CAN 帧并发送。
// 当前先实现 DJI 0x200 组（底盘 4x3508 常用）。

class CanMotorTxTask {
public:
    bool Start();

    // Backward-compatible name
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
