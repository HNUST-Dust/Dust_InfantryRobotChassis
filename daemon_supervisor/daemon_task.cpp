/**
 * @file daemon_task.cpp
 * @brief CMSIS-RTOS2 守护任务实现
 * 
 * 设计思路：
 * =========
 * 守护检测需要在独立的 RTOS 任务中周期运行，原因：
 * 
 * 1. 独立性：不依赖任何被监控模块的任务，即使其他任务卡死，守护任务仍能检测
 * 2. 实时性：周期性 tick 确保超时检测的时间精度
 * 3. 优先级：可以设置较高优先级，确保检测不被延迟
 * 
 * 时间源选择：
 * ──────────
 * 使用 DWT (Data Watchpoint and Trace) 计时器获取毫秒时间戳。
 * DWT 是 ARM Cortex-M 内核的硬件计时器，特点：
 * - 不依赖 RTOS tick
 * - 高精度（通常是 CPU 时钟周期级）
 * - 不需要额外的定时器外设
 * 
 * 也可以改用 osKernelGetTickCount() 获取 RTOS tick，但 DWT 更精确。
 */

#include "supervisor.hpp"
#include "cmsis_os2.h"
#include "bsp_dwt.h"  // DWT bsp - 提供高精度时间戳

// -----------------------------
// CMSIS-RTOS2 daemon task
// -----------------------------

/**
 * @brief 守护任务主函数
 * 
 * 作为独立的 RTOS 任务运行，周期性检测所有客户端的健康状态。
 * 
 * @param arg 任务参数（未使用，保持接口兼容性）
 * 
 * 任务特性：
 * ──────────
 * - 周期：10ms (100Hz)
 * - 推荐优先级：中高（高于普通应用任务，低于关键实时任务）
 * - 栈大小：较小即可（约 256-512 字节），主要是函数调用开销
 * 
 * 为什么选择 10ms 周期？
 * ────────────────────
 * 1. 10ms 足够检测大多数超时（通常 > 30ms）
 * 2. 不会造成过大的 CPU 开销
 * 3. 是嵌入式系统中常用的控制周期
 * 
 * 可以根据系统需求调整周期：
 * - 更短周期（如 1ms）：更快的超时响应，但 CPU 开销更大
 * - 更长周期（如 100ms）：更低的 CPU 开销，但超时检测精度下降
 */
void daemon_task(void* arg) {
    (void)arg;  // 显式标记参数未使用，消除编译器警告

    // 主循环 - 永不退出
    while (1) {
        // 获取当前毫秒数，转换为 uint32_t 用于 wrap-around 安全比较
        // dwt_get_timeline_ms() 返回自系统启动以来的毫秒数
        uint32_t now_ms = static_cast<uint32_t>(dwt_get_timeline_ms());

        // 调用 DaemonSupervisor 的核心检测函数
        // 这会检查所有客户端，触发超时回调，喂硬件看门狗
        DaemonSupervisor::tick(now_ms);

        // 等待 10ms -> 100Hz 检测频率
        // osDelay 会让出 CPU，允许其他任务运行
        osDelay(10);
    }
    
    // 注意：正常情况下永远不会到达这里
    // 如果需要支持任务退出，可以添加退出条件和清理代码
}
