/**
 * @file topic_notify.hpp
 * @brief Topic 发布事件 → RTOS 唤醒机制（Notifier）
 *
 * 设计思路：
 * =========
 * Topic/RingTopic 的数据读取通常是“拉取式”（订阅者 copy）。
 * `Notifier` 提供“推送式唤醒”：当发布者 publish 新数据时，触发 RTOS EventFlags，
 * 让订阅者任务从阻塞态唤醒后再去拉取数据。
 *
 * 线程模型：
 * =========
 * - `notify()` 只做 EventFlags set；不做数据拷贝。
 * - 是否可在 ISR 调用取决于底层 CMSIS-RTOS2 实现对 EventFlagsSet 的支持。
 *
 * 注意事项：
 * =========
 * - Notifier 的生命周期必须覆盖 Topic 注册期间（通常放在任务静态对象中）。
 * - EventFlags mask 的分配建议集中管理，避免不同事件源冲突。
 */

#pragma once

#include "cmsis_os2.h"

/*******************************************************************************
 * Notifier - RTOS 事件通知器
 * ----------------------------------------------------------------------------
 *
 * 【功能】
 *   封装 RTOS 事件标志，用于在数据发布时唤醒等待的任务。
 *   实现了"推送"通知机制，配合 Topic 的"拉取"读取。
 *
 * 【设计目的】
 *   在纯轮询模式下，订阅者需要不断检查是否有新数据，浪费 CPU。
 *   通过 Notifier，发布者可以在有新数据时主动通知订阅者，
 *   实现事件驱动的高效架构。
 *
 * 【工作原理】
 *   1. 订阅者任务创建 EventFlags 并等待
 *   2. 将 Notifier 注册到 RingTopic
 *   3. 发布者 publish() 后自动调用 notify()
 *   4. 订阅者被唤醒，读取数据
 *
 * 【典型使用流程】
 *
 *   // 1. 创建事件标志（通常在任务初始化时）
 *   osEventFlagsId_t evt = osEventFlagsNew(NULL);
 *   #define IMU_DATA_FLAG 0x01
 *
 *   // 2. 创建 Notifier 并注册到 Topic
 *   Notifier imu_notifier(evt, IMU_DATA_FLAG);
 *   orb::imu[0].register_notifier(&imu_notifier);
 *
 *   // 3. 订阅者任务等待事件
 *   void ImuTask(void* arg) {
 *       RingSub<ImuData, 4> sub(orb::imu[0]);
 *       ImuData data;
 *
 *       while (true) {
 *           // 等待事件（阻塞，不消耗 CPU）
 *           osEventFlagsWait(evt, IMU_DATA_FLAG, osFlagsWaitAny, osWaitForever);
 *
 *           // 被唤醒后读取所有新数据
 *           while (sub.copy(data)) {
 *               process_imu(data);
 *           }
 *       }
 *   }
 *
 *   // 4. 发布者发布数据（自动触发通知）
 *   void ImuISR() {
 *       ImuData data = read_imu_hardware();
 *       orb::imu[0].publish(data);  // 内部调用 notifier->notify()
 *   }
 *
 * 【注意事项】
 *   - Notifier 对象的生命周期必须长于使用它的 Topic
 *   - 可以在 ISR 中安全调用 notify()（osEventFlagsSet 是 ISR 安全的）
 *   - 一个 EventFlags 可以有多个 mask，用于区分不同的事件源
 *
 ******************************************************************************/
class Notifier {
public:
    /***************************************************************************
     * 构造函数
     * -------------------------------------------------------------------------
     * @param evt  RTOS 事件标志句柄（由 osEventFlagsNew 创建）
     * @param mask 要设置的事件位掩码（用于区分不同事件）
     **************************************************************************/
    explicit Notifier(osEventFlagsId_t evt, uint32_t mask)
        : evt_(evt), mask_(mask)
    {}

    /***************************************************************************
     * notify - 发送通知
     * -------------------------------------------------------------------------
     * 【行为】
     *   设置事件标志位，唤醒正在等待该事件的任务
     *
     * 【线程安全】
     *   - 可以在普通任务中调用
     *   - 可以在 ISR 中调用（osEventFlagsSet 是 ISR 安全的）
     **************************************************************************/
    inline void notify()
    {
        osEventFlagsSet(evt_, mask_);
    }

private:
    osEventFlagsId_t evt_;  // RTOS 事件标志句柄
    uint32_t mask_;         // 事件位掩码
};

