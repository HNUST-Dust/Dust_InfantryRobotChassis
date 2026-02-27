/**
 * @file daemon_client.hpp
 * @brief 守护客户端类 - 被监控模块的抽象表示
 * 
 * 设计思路：
 * =========
 * DaemonClient 代表一个需要被监控的模块/组件。每个模块创建自己的 DaemonClient 实例，
 * 并定期调用 feed() 方法来表明自己仍在正常运行（类似"喂狗"）。
 * 
 * 如果模块在指定的超时时间内没有 feed()，DaemonSupervisor 会将其标记为 OFFLINE，
 * 并根据配置的故障等级触发相应的处理。
 * 
 * 使用场景示例：
 * - IMU 传感器模块：每次成功读取数据后 feed()
 * - 通信模块：每次成功收发数据后 feed()
 * - 控制模块：每次控制周期完成后 feed()
 */

#pragma once
#include <cstdint>

class DaemonClient {

public:
    /**
     * @brief 客户端状态机
     * 
     * 状态转换图：
     *   INIT ──feed()──> ONLINE ──timeout──> OFFLINE ──feed()──> RECOVERING ──feed()──> ONLINE
     *                      ↑                                                              │
     *                      └──────────────────────────────────────────────────────────────┘
     * 
     * - INIT:       初始状态，刚创建但还未 feed() 过
     * - ONLINE:     正常在线，模块工作正常
     * - OFFLINE:    离线，超时未收到 feed()，触发故障回调
     * - RECOVERING: 恢复中，从 OFFLINE 收到 feed() 后的过渡状态
     */
    enum class State : uint8_t {
        INIT,       // 初始化状态 - 尚未首次喂狗
        ONLINE,     // 在线状态 - 模块正常运行
        OFFLINE,    // 离线状态 - 超时未喂狗，已触发故障处理
        RECOVERING  // 恢复状态 - 从离线状态收到喂狗，正在恢复
    };

    /**
     * @brief 模块所属域/分类
     * 
     * 用于对模块进行分类，便于：
     * 1. 日志和调试时快速识别模块类型
     * 2. 未来可能的分域故障处理策略
     * 3. 统计各域的健康状态
     */
    enum class Domain : uint8_t {
        SENSOR,     // 传感器域 - IMU、编码器、遥控器等
        CONTROL,    // 控制域 - 底盘控制、云台控制等
        COMM,       // 通信域 - CAN、串口、裁判系统等
        POWER,      // 电源域 - 电池监控、电源管理等
        GENERIC     // 通用/其他
    };

    /**
     * @brief 故障严重等级
     * 
     * 决定了模块离线时的处理方式：
     * - WARN:     仅警告，不影响系统运行
     * - DEGRADED: 降级运行，部分功能受限
     * - FATAL:    致命错误，可能触发系统级故障处理（如紧急停止）
     */
    enum class FaultLevel : uint8_t {
        WARN,       // 警告级 - 记录日志，继续运行
        DEGRADED,   // 降级级 - 进入降级模式，限制部分功能
        FATAL       // 致命级 - 触发系统故障钩子，可能导致系统停止
    };

    /**
     * @brief 模块优先级
     * 
     * CRITICAL 优先级的模块会影响硬件看门狗：
     * 只有所有 CRITICAL 模块都在线时，才会喂硬件看门狗。
     * 这确保了关键模块故障时，硬件看门狗会触发系统复位。
     */
    enum class Priority : uint8_t {
        LOW,        // 低优先级 - 非关键模块
        NORMAL,     // 普通优先级 - 标准模块
        HIGH,       // 高优先级 - 重要模块
        CRITICAL    // 关键优先级 - 影响硬件看门狗喂狗
    };


    /**
     * @brief 故障回调函数类型
     * @param client 触发故障的客户端引用，可通过 owner() 获取原始模块指针
     */
    using Callback = void(*)(DaemonClient&);

    /**
     * @brief 构造函数
     * 
     * @param timeout_ms 超时时间(毫秒) - 超过此时间未 feed() 则判定为离线
     * @param cb         故障回调函数 - 模块离线时调用
     * @param owner      拥有者指针 - 指向实际的模块对象，便于回调中访问
     * @param domain     所属域 - 模块分类
     * @param level      故障等级 - 决定离线时的处理方式
     * @param priority   优先级 - CRITICAL 会影响硬件看门狗
     */
    DaemonClient(uint32_t timeout_ms,
                Callback cb,
                void *owner,
                Domain domain,
                FaultLevel level,
                Priority priority
            );

    /**
     * @brief 喂狗/心跳函数
     * 
     * 模块应定期调用此函数表明自己正常运行。
     * 调用频率应高于 1000/timeout_ms Hz，建议至少 2-3 倍余量。
     * 
     * @param now_ms 当前时间戳(毫秒) - 用于记录最后活跃时间
     * 
     * 状态转换：
     * - INIT -> ONLINE (首次喂狗)
     * - OFFLINE -> RECOVERING (离线后恢复)
     * - RECOVERING -> ONLINE (确认恢复)
     * - ONLINE -> ONLINE (保持在线)
     */
    void feed(uint32_t now_ms);


    /** @brief 获取当前状态 */
    State state() const { return state_; }
    
    /** @brief 获取拥有者指针 - 可转换回原始模块类型 */
    void* owner() const { return owner_; }

private:
    // DaemonSupervisor 需要访问私有成员进行超时检测
    friend class DaemonSupervisor;

    // ===== 成员变量 (使用 snake_case_ 命名风格) =====
    
    uint32_t timeout_ms_;    // 超时阈值(毫秒)
    uint32_t last_seen_ms_;  // 最后一次 feed() 的时间戳
    Callback callback_;      // 故障回调函数指针
    void* owner_;            // 拥有者/关联模块指针

    State state_;            // 当前状态
    Domain domain_;          // 所属域
    FaultLevel level_;       // 故障等级
    Priority priority_;      // 优先级

};
