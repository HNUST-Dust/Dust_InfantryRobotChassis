/**
 * @file daemon_client.cpp
 * @brief DaemonClient 实现
 * 
 * 核心逻辑：
 * =========
 * DaemonClient 的主要职责是维护自身的状态机，通过 feed() 方法更新状态。
 * 超时检测由 DaemonSupervisor 负责，Client 只负责记录时间戳和状态转换。
 */

#include "daemon_client.hpp"

/**
 * @brief 构造函数实现
 * 
 * 初始化所有成员变量，初始状态为 INIT。
 * last_seen_ms_ 初始化为 0，这意味着如果模块创建后长时间不 feed()，
 * 会在第一次 tick 时就被判定为超时（取决于当前时间）。
 * 
 * 建议：创建后尽快调用 feed() 进行首次喂狗。
 */
DaemonClient::DaemonClient(uint32_t timeout_ms,
                           Callback cb,
                           void *owner,
                           Domain domain,
                           FaultLevel level,
                           Priority priority)
: timeout_ms_(timeout_ms),
  last_seen_ms_(0),           // 初始时间戳为 0
  callback_(cb),
  owner_(owner),
  domain_(domain),
  level_(level),
  priority_(priority),
  state_(State::INIT)         // 初始状态
{
    // 构造函数体为空，所有初始化通过初始化列表完成
    // 这是 C++ 的最佳实践，效率更高
}


/**
 * @brief 喂狗函数实现
 * 
 * 这是模块表明"我还活着"的唯一方式。
 * 
 * @param now_ms 当前时间戳，通常来自系统时钟或 DWT 计时器
 * 
 * 状态机转换逻辑：
 * ────────────────
 * 1. INIT -> ONLINE: 
 *    首次喂狗，模块正式上线
 * 
 * 2. OFFLINE -> RECOVERING:
 *    模块曾经离线，现在恢复了心跳
 *    进入 RECOVERING 而非直接 ONLINE，提供一个过渡状态
 *    可用于实现"需要连续多次喂狗才算真正恢复"的逻辑
 * 
 * 3. RECOVERING -> ONLINE:
 *    确认恢复，回到正常在线状态
 * 
 * 4. ONLINE -> ONLINE:
 *    正常情况，保持在线，只更新时间戳
 * 
 * 时间戳说明：
 * ──────────
 * now_ms 使用 uint32_t，大约 49.7 天会溢出回绕。
 * 但由于 tick() 中使用无符号减法 (now_ms - last_seen_ms_)，
 * 即使发生回绕，只要间隔不超过 24.8 天，计算仍然正确。
 */
void DaemonClient::feed(uint32_t now_ms)
{
    // 更新最后活跃时间戳
    last_seen_ms_ = now_ms;

    // 根据当前状态进行状态转换
    switch (state_) {

    case State::INIT:
        // 首次喂狗：INIT -> ONLINE
        state_ = State::ONLINE;
        break;

    case State::OFFLINE:
        // 从离线恢复：OFFLINE -> RECOVERING
        // 不直接进入 ONLINE，提供恢复过渡期
        state_ = State::RECOVERING;
        break;

    case State::RECOVERING:
        // 确认恢复：RECOVERING -> ONLINE
        // 连续第二次喂狗，确认模块稳定恢复
        state_ = State::ONLINE;
        break;

    default:
        // State::ONLINE 或其他：保持当前状态
        // 只更新时间戳（已在函数开头完成）
        break;
    }
}
