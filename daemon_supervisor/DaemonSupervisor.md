Daemon Supervisor 框架说明
<p align="right">noneofever@gmail.com</p>
1. 概述

Daemon Supervisor 是一个用于监控模块运行状态、处理模块故障、提供软件与硬件看门狗接口的模块。

其设计思想来源于 PX4 的 detect task / health manager，适用于机器人、飞控系统等嵌入式平台。

核心目标：

监控各模块心跳（heartbeat）

自动触发模块离线回调

支持模块状态机（生命周期管理）

支持优先级、故障等级与系统级安全机制

可扩展硬件看门狗、LED/蜂鸣器提示和降级模式

2. 架构与模块关系
2.1 文件结构
daemon/
├── daemon_client.hpp       // Client类定义
├── daemon_client.cpp       // Client类实现
├── supervisor.hpp          // Supervisor类定义
├── supervisor.cpp          // Supervisor实现
├── daemon_task.cpp         // RTOS任务 glue

2.2 模块关系
[DaemonClient]  →  [DaemonSupervisor]  →  [System / HW Watchdog]

每个模块实例拥有一个 DaemonClient
Supervisor 管理所有 Client
Client 离线 → 回调 → 触发系统安全策略 / HW feed

3. DaemonClient（模块客户端）
3.1 数据结构
class DaemonClient {
public:
    enum class State { INIT, ONLINE, OFFLINE, RECOVERING };
    enum class Domain { SENSOR, CONTROL, COMM, POWER, GENERIC };
    enum class FaultLevel { WARN, DEGRADED, FATAL };
    enum class Priority { LOW, NORMAL, HIGH, CRITICAL };

    using Callback = void(*)(DaemonClient&);

private:
    uint32_t _timeout;
    uint32_t _last_seen;
    Callback _callback;
    void* _owner;
    State _state;
    Domain _domain;
    FaultLevel _level;
    Priority _priority;
};

3.2 生命周期
INIT → ONLINE → OFFLINE → RECOVERING → ONLINE


INIT：模块初始化未开始喂狗

ONLINE：模块正常工作

OFFLINE：模块离线/超时

RECOVERING：模块重新上线，恢复正常

3.3 核心方法
// 构造函数
DaemonClient(timeout_ms, callback, owner, domain, level, priority);

// 喂狗（更新 last_seen）
void feed(uint32_t now_ms);

// 查询状态
State state() const;
void* owner() const;

4. DaemonSupervisor（中央守护者）
4.1 数据结构
class DaemonSupervisor {
public:
    static constexpr int MAX_CLIENTS = 16;

    static void init();
    static DaemonClient* register_client(DaemonClient* client);
    static void tick(uint32_t now_ms);

    using SystemFaultHook = void(*)(DaemonClient&);
    static void set_system_fault_hook(SystemFaultHook hook);
    static void set_hw_feed(HwFeed f);

private:
    static DaemonClient* _clients[MAX_CLIENTS];
    static int _count;
    static SystemFaultHook _system_hook;
    static HwFeed _hw_feed;

    static bool _critical_alive();
};

4.2 功能

管理所有 Client

定期检查每个 Client 是否超时

oneshot 离线回调

系统级 FATAL 回调

优先级 CRITICAL Client 控制 HW watchdog feed

5. 使用方法
5.1 注册客户端
DaemonClient imu_daemon(
    20,                  // timeout ms
    imu_fault,           // 离线回调
    &imu,                // parent pointer
    DaemonClient::Domain::SENSOR,
    DaemonClient::FaultLevel::DEGRADED,
    DaemonClient::Priority::NORMAL
);

DaemonSupervisor::register_client(&imu_daemon);

5.2 喂狗
imu_daemon.feed(get_sys_ms());

5.3 定义回调
void imu_fault(DaemonClient &c)
{
    auto *imu_ptr = static_cast<IMU*>(c.owner());
    imu_ptr->reset();
}

5.4 系统级 FATAL hook
void system_fault(DaemonClient &c)
{
    NVIC_SystemReset(); // MCU 重启
}

DaemonSupervisor::set_system_fault_hook(system_fault);

5.5 硬件看门狗绑定
extern "C" void stm32_iwdg_feed();
DaemonSupervisor::set_hw_feed(stm32_iwdg_feed);


仅在所有 CRITICAL Client ONLINE 时喂硬件看门狗

6. 任务调度（RTOS glue）
void daemon_task(void*)
{
    while(true) {
        DaemonSupervisor::tick(get_sys_ms());
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}


推荐使用 100Hz 或 1kHz 任务频率

每次 tick 遍历所有 Client 更新状态并触发回调

7. 高级功能（可扩展）
功能	作用
Domain Aggregation	统计 domain 内模块状态，实现子系统级健康判断
Degrade Mode	当 DEGRADED client 出现，系统进入降级模式而非直接 reboot
LED / Buzzer Binding	将模块状态映射到物理 LED/蜂鸣器，实现可视化反馈
Fault History Ringbuffer	记录历史故障，支持事后分析和 sporadic bug 调试

这些功能可以在 Supervisor tick 或 callback 中拓展。

8. 框图
+-----------------------------+
|         RTOS Task           |
|  daemon_task() 100Hz        |
+-----------------------------+
            |
            v
+-----------------------------+
|      DaemonSupervisor       |
|-----------------------------|
| _clients[]                  |
| tick(now_ms)                |
| system_hook                 |
| hw_feed                     |
+-----------------------------+
     |         |       |
     v         v       v
+--------+ +--------+ +--------+
| Client | | Client | | Client |
| IMU    | | Motor  | | Comm   |
+--------+ +--------+ +--------+
     |         |       |
 callback    callback callback