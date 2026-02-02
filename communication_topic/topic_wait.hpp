/**
 * @file topic_wait.hpp
 * @brief 任务侧事件驱动工具：TopicWaiter（EventFlags + Notifier + wait）
 *
 * 设计思路：
 * =========
 * `TopicWaiter` 将“创建 EventFlags + 持有 Notifier + 等待事件”的样板代码收敛成一个小工具。
 * 常见用法：
 * - 任务创建 waiter
 * - 将 `waiter.notifier()` 注册到 Topic/RingTopic
 * - 任务循环中 `waiter.wait()` 阻塞等待发布事件
 *
 * 边界：
 * =====
 * - 仅负责唤醒，不负责搬运数据；数据仍由 `Sub/RingSub` copy。
 * - 采用静态内存（EventFlags cb_mem），避免动态分配。
 */

#pragma once

#include "cmsis_os2.h"
#include "topic_notify.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "event_groups.h"
}

// TopicWaiter: 将 Notifier + EventFlags 等待封装成一个小工具，便于任务侧事件驱动。
// 适用：RingTopic 的 register_notifier() 场景。

class TopicWaiter {
public:
    TopicWaiter() = default;

    // mask 默认 bit0
    explicit TopicWaiter(uint32_t mask)
        : mask_(mask)
    {
        (void)Init(mask);
    }

    // 允许默认构造后再初始化
    bool Init(uint32_t mask = (1u << 0))
    {
        if (evt_ != nullptr) {
            // 不允许重复 Init（避免多次注册/泄漏）
            configASSERT(false);
            return false;
        }

        mask_ = mask;
        evt_attr_ = osEventFlagsAttr_t{
            .name = "topic_waiter_evt",
            .cb_mem = &evt_cb_,
            .cb_size = sizeof(evt_cb_),
        };
        evt_ = osEventFlagsNew(&evt_attr_);
        if (evt_ == nullptr) {
            configASSERT(false);
            return false;
        }

        notifier_ = Notifier(evt_, mask_);
        return true;
    }

    Notifier* notifier() { return &notifier_; }
    const Notifier* notifier() const { return &notifier_; }

    osEventFlagsId_t evt() const { return evt_; }
    uint32_t mask() const { return mask_; }

    void wait(uint32_t timeout = osWaitForever) const
    {
        (void)osEventFlagsWait(evt_, mask_, osFlagsWaitAny, timeout);
    }

private:
    osEventFlagsId_t evt_ = nullptr;
    uint32_t mask_ = 1u << 0;

    StaticEventGroup_t evt_cb_{};
    osEventFlagsAttr_t evt_attr_{};

    Notifier notifier_{nullptr, 0};
};
