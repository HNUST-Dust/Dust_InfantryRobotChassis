/**
 * @file supervisor.cpp
 * @brief DaemonSupervisor 实现
 * 
 * 核心算法：
 * =========
 * 超时检测使用无符号整数减法，天然支持时间戳回绕：
 *   elapsed = now_ms - last_seen_ms_
 * 
 * 即使 now_ms 溢出回绕到 0，只要实际间隔不超过 uint32_t 最大值的一半
 * （约 24.8 天），计算结果仍然正确。
 * 
 * 例如：last_seen = 0xFFFFFFF0, now = 0x00000010
 *       elapsed = 0x00000010 - 0xFFFFFFF0 = 0x00000020 = 32ms ✓
 */

#include "supervisor.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

namespace {
osThreadId_t s_daemon_thread_id = nullptr;
StaticTask_t s_daemon_tcb{};
StackType_t s_daemon_stack[256]{}; // 256 * 4 bytes = 1KB
} // namespace


// ===== 静态成员变量定义 =====
// 注意：静态成员必须在类外定义，否则链接时会报未定义错误

/** @brief 系统故障钩子，初始为空 */
DaemonSupervisor::SystemFaultHook DaemonSupervisor::system_hook_ = nullptr;

/** @brief 硬件看门狗喂狗函数，初始为空 */
DaemonSupervisor::HwFeed DaemonSupervisor::hw_feed_ = nullptr;

/** @brief 客户端指针数组 */
DaemonClient* DaemonSupervisor::clients_[MAX_CLIENTS];

/** @brief 已注册客户端计数 */
int DaemonSupervisor::count_ = 0;


/**
 * @brief 初始化函数实现
 * 
 * 简单重置计数器。由于 clients_ 数组通过计数器访问，
 * 不需要显式清空数组内容。
 */
void DaemonSupervisor::init()
{
    count_ = 0;
    // 可选：显式清空数组（防御性编程）
    // for (int i = 0; i < MAX_CLIENTS; i++) clients_[i] = nullptr;
}


/**
 * @brief 注册客户端实现
 * 
 * 将客户端指针添加到数组末尾。
 * 时间复杂度 O(1)，空间复杂度 O(1)。
 * 
 * @note 没有去重检查，重复注册同一客户端会导致多次检测
 * @note 没有删除功能，一旦注册就永久存在（RTOS 环境下通常不需要动态删除）
 */
DaemonClient* DaemonSupervisor::register_client(DaemonClient *client)
{
    // 检查是否超过最大客户端数量
    if (count_ >= MAX_CLIENTS) return nullptr;

    // 添加到数组并递增计数
    clients_[count_++] = client;
    return client;
}


/**
 * @brief 周期心跳检测实现
 * 
 * 这是整个守护系统的核心逻辑。
 * 
 * 执行步骤：
 * ──────────
 * 1. 遍历所有客户端
 * 2. 对每个客户端计算: elapsed_time = now_ms - last_seen_ms
 * 3. 如果 elapsed_time > timeout_ms，说明超时
 * 4. 超时且当前非 OFFLINE 状态：
 *    a. 将状态设为 OFFLINE
 *    b. 调用客户端的回调函数（如果设置了）
 *    c. 如果是 FATAL 级别，调用系统故障钩子（如果设置了）
 * 5. 最后，如果所有 CRITICAL 客户端都在线，喂硬件看门狗
 * 
 * 为什么检查 "state != OFFLINE"？
 * ────────────────────────────
 * 避免重复触发回调。一旦进入 OFFLINE 状态，只有 feed() 才能改变状态。
 * 如果不检查，每次 tick 都会触发回调，造成刷屏和性能浪费。
 * 
 * @param now_ms 当前时间戳（毫秒），通常来自 DWT 或系统 tick
 */
void DaemonSupervisor::tick(uint32_t now_ms)
{
    // ===== 第一步：遍历检查所有客户端 =====
    for (int i = 0; i < count_; i++) {

        auto *c = clients_[i];
        
        // 防御性检查：跳过空指针（理论上不应该发生）
        if (!c) continue;
        
        // 计算距离上次喂狗的时间间隔
        // 使用无符号减法，自动处理时间戳回绕
        uint32_t elapsed = now_ms - c->last_seen_ms_;
        
        // 检查是否超时
        if (elapsed > c->timeout_ms_) {

            // 仅当状态不是 OFFLINE 时才处理（避免重复触发）
            if (c->state_ != DaemonClient::State::OFFLINE) {

                // 设置状态为离线
                c->state_ = DaemonClient::State::OFFLINE;

                // 触发客户端级别的回调（如果设置了）
                // 用于模块特定的故障处理
                if (c->callback_)
                    c->callback_(*c);

                // 如果是 FATAL 级别，还要触发系统级故障钩子
                // 用于全局紧急处理（如停止电机、进入安全模式）
                if (c->level_ == DaemonClient::FaultLevel::FATAL && system_hook_)
                    system_hook_(*c);
            }
            // else: 已经是 OFFLINE，不重复处理

        }
        // else: 未超时，客户端正常，无需处理
    }
    
    // ===== 第二步：检查是否喂硬件看门狗 =====
    // 只有当：
    // 1. 设置了硬件看门狗喂狗函数
    // 2. 所有 CRITICAL 优先级的客户端都在线
    // 才会喂硬件看门狗
    if (hw_feed_ && critical_alive_())
        hw_feed_();

}


/**
 * @brief 设置系统故障钩子
 */
void DaemonSupervisor::set_system_fault_hook(SystemFaultHook hook)
{
    system_hook_ = hook;
}

osThreadId_t DaemonSupervisor::Start(SystemFaultHook hook)
{
    set_system_fault_hook(hook);
    return Start(ThreadConfig{});
}

osThreadId_t DaemonSupervisor::Start(SystemFaultHook hook, ThreadConfig cfg)
{
    set_system_fault_hook(hook);
    return Start(cfg);
}

osThreadId_t DaemonSupervisor::Start()
{
    return Start(ThreadConfig{});
}

osThreadId_t DaemonSupervisor::Start(ThreadConfig cfg)
{
    // Must be called before any module registers clients.
    DaemonSupervisor::init();

    if (s_daemon_thread_id != nullptr) {
        return s_daemon_thread_id;
    }

    const osThreadAttr_t attr = {
        .name = cfg.name,
        .cb_mem = &s_daemon_tcb,
        .cb_size = sizeof(s_daemon_tcb),
        .stack_mem = s_daemon_stack,
        .stack_size = sizeof(s_daemon_stack),
        .priority = cfg.priority,
    };

    s_daemon_thread_id = osThreadNew(daemon_task, nullptr, &attr);
    return s_daemon_thread_id;
}


/**
 * @brief 检查所有关键客户端是否在线
 * 
 * 遍历所有客户端，检查 CRITICAL 优先级的客户端状态。
 * 
 * @return true  所有 CRITICAL 客户端都是 ONLINE
 * @return false 存在 CRITICAL 客户端不是 ONLINE（包括 INIT、OFFLINE、RECOVERING）
 * 
 * 设计考量：
 * ──────────
 * - INIT 状态不算在线：模块必须至少 feed() 一次才算真正工作
 * - RECOVERING 状态不算在线：需要稳定恢复后才信任
 * - 这种保守策略确保系统安全
 */
bool DaemonSupervisor::critical_alive_()
{
    for (int i = 0; i < count_; i++) {
        auto *c = clients_[i];

        // 检查 CRITICAL 优先级的客户端
        if (c->priority_ == DaemonClient::Priority::CRITICAL &&
            c->state_ != DaemonClient::State::ONLINE)
            return false;  // 发现一个不在线的 CRITICAL 客户端
    }
    return true;  // 所有 CRITICAL 客户端都在线（或没有 CRITICAL 客户端）
}


/**
 * @brief 设置硬件看门狗喂狗函数
 */
void DaemonSupervisor::set_hw_feed(HwFeed f)
{
    hw_feed_ = f;
}
