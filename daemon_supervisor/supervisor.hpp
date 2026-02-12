/**
 * @file supervisor.hpp
 * @brief 守护监督者 - 集中管理所有 DaemonClient 的核心类
 * 
 * 设计思路：
 * =========
 * DaemonSupervisor 是整个守护系统的核心，采用单例模式（通过静态成员实现）。
 * 它负责：
 * 1. 管理所有注册的 DaemonClient
 * 2. 周期性检查所有客户端的超时状态
 * 3. 触发故障回调和系统级故障钩子
 * 4. 控制硬件看门狗的喂狗（仅当关键模块都在线时）
 * 
 * 架构图：
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    DaemonSupervisor                         │
 * │  ┌─────────┐ ┌─────────┐ ┌─────────┐       ┌─────────┐     │
 * │  │ Client1 │ │ Client2 │ │ Client3 │  ...  │ ClientN │     │
 * │  │  (IMU)  │ │ (Motor) │ │ (Comm)  │       │         │     │
 * │  └────┬────┘ └────┬────┘ └────┬────┘       └────┬────┘     │
 * │       │           │           │                  │          │
 * │       └───────────┴───────────┴──────────────────┘          │
 * │                           │                                  │
 * │                      tick(now_ms)                           │
 * │                           │                                  │
 * │           ┌───────────────┼───────────────┐                 │
 * │           ▼               ▼               ▼                 │
 * │    [超时检测]      [回调触发]      [硬件看门狗]              │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * 使用流程：
 * 1. DaemonSupervisor::init() - 初始化
 * 2. 创建 DaemonClient 实例
 * 3. DaemonSupervisor::register_client() - 注册客户端
 * 4. 设置回调钩子（可选）
 * 5. 在独立任务中周期调用 tick()
 */

#pragma once
#include "daemon_client.hpp"

#include "cmsis_os2.h"


class DaemonSupervisor {

public:
    /** @brief 最大支持的客户端数量 - 根据系统需求调整 */
    static constexpr int MAX_CLIENTS = 16;

    /**
     * @brief 初始化守护监督者
     * 
     * 必须在注册任何客户端之前调用。
     * 重置客户端计数器，清空客户端列表。
     */
    static void init();

    /**
     * @brief 注册一个守护客户端
     * 
     * @param client 指向 DaemonClient 的指针（生命周期需由调用者管理）
     * @return 成功返回 client 指针，失败（超过 MAX_CLIENTS）返回 nullptr
     * 
     * 注意：
     * - client 指针必须在整个监控期间保持有效
     * - 不要传入栈上的临时对象
     * - 推荐使用全局变量或静态变量
     */
    static DaemonClient* register_client(DaemonClient *client);

    /**
     * @brief 周期性心跳检测函数
     * 
     * 这是守护系统的核心函数，应在独立的 RTOS 任务中周期调用。
     * 
     * @param now_ms 当前时间戳(毫秒)
     * 
     * 执行流程：
     * 1. 遍历所有已注册的客户端
     * 2. 检查每个客户端是否超时 (now_ms - last_seen > timeout)
     * 3. 超时且非离线状态 -> 设为 OFFLINE，触发回调
     * 4. 如果是 FATAL 级别，还会触发系统故障钩子
     * 5. 最后检查是否应喂硬件看门狗
     * 
     * 调用频率建议：10ms (100Hz)，确保超时检测的精度
     */
    static void tick(uint32_t now_ms);
    
    /**
     * @brief 系统级故障钩子类型
     * 
     * 当 FATAL 级别的客户端离线时调用。
     * 可用于：
     * - 紧急停止电机
     * - 切换到安全模式
     * - 触发系统复位
     * - 发送告警信息
     */
    using SystemFaultHook = void(*)(DaemonClient&);

    /**
     * @brief 设置系统级故障钩子
     * 
     * @param hook 故障处理函数指针，设为 nullptr 可禁用
     */
    static void set_system_fault_hook(SystemFaultHook hook);

    /**
     * @brief daemon_task 线程启动参数
     *
     * 默认值与工程内其它模块 Start() 的优先级风格保持一致。
     */
    struct ThreadConfig {
        const char* name;
        osPriority_t priority;

        constexpr ThreadConfig(
            const char* n = "daemon",
            osPriority_t p = (osPriority_t)osPriorityAboveNormal)
            : name(n)
            , priority(p)
        {
        }
    };

    /**
     * @brief 初始化 DaemonSupervisor 并启动 daemon_task（静态内存）
     *
     * 约束：必须在任何模块注册 DaemonClient 前调用。
     * 提示：系统级故障钩子请先通过 set_system_fault_hook() 绑定。
     * @return 创建/已存在的线程句柄；失败返回 nullptr
     */
    static osThreadId_t Start();

    /**
     * @brief 同 Start()，可自定义线程属性。
     */
    static osThreadId_t Start(ThreadConfig cfg);

    /**
     * @brief 兼容接口：绑定 hook 后启动（等价于 set_system_fault_hook(hook); Start();）
     */
    static osThreadId_t Start(SystemFaultHook hook);

    /**
     * @brief 兼容接口：绑定 hook 后启动并自定义线程属性。
     */
    static osThreadId_t Start(SystemFaultHook hook, ThreadConfig cfg);

    /**
     * @brief 硬件看门狗喂狗函数类型
     * 
     * 只有当所有 CRITICAL 优先级的客户端都在线时才会调用。
     * 这确保了关键模块故障时，硬件看门狗会超时触发系统复位。
     */
    using HwFeed = void(*)();

    /**
     * @brief 设置硬件看门狗喂狗函数
     * 
     * @param f 喂狗函数指针，通常调用硬件看门狗外设的喂狗操作
     * 
     * 示例：
     *   void hw_watchdog_feed() {
     *       HAL_IWDG_Refresh(&hiwdg);  // 喂独立看门狗
     *   }
     *   DaemonSupervisor::set_hw_feed(hw_watchdog_feed);
     */
    static void set_hw_feed(HwFeed f);

private:
    // ===== 静态成员变量 =====
    
    /** @brief 客户端指针数组 - 存储所有注册的客户端 */
    static DaemonClient* clients_[MAX_CLIENTS];
    
    /** @brief 当前已注册的客户端数量 */
    static int count_;
    
    /** @brief 系统故障钩子函数指针 */
    static SystemFaultHook system_hook_;
    
    /** @brief 硬件看门狗喂狗函数指针 */
    static HwFeed hw_feed_;

    /**
     * @brief 检查所有关键客户端是否都在线
     * 
     * @return true  所有 CRITICAL 优先级的客户端都是 ONLINE 状态
     * @return false 存在 CRITICAL 客户端不在线
     * 
     * 用于决定是否喂硬件看门狗。
     */
    static bool critical_alive_();

};

/**
 * @brief CMSIS-RTOS2 守护任务入口函数
 * 
 * 作为独立的 RTOS 任务运行，周期调用 DaemonSupervisor::tick()
 * 
 * @param arg 任务参数（未使用）
 * 
 * 使用方式：
 *   osThreadNew(daemon_task, nullptr, nullptr);
 */
void daemon_task(void*);