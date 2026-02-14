/**
 * @file dvc_referee.h
 * @brief 裁判系统设备封装：UART RX 解析 -> 发布 Topic；必要时驱动 UI 更新。
 *
 * **定位**
 * - 该模块负责对接裁判系统串口数据（Referee），并把解析结果以 Topic 的形式提供给业务层。
 * - 仅解析本工程实际用到的字段（例如 HP、功率限制、发射信息）。
 *
 * **数据流**
 * - 输入：
 *   - UART RX 回调 `RxCpltCallback()`（来自 Platform/UART port 的 RX 完成回调路径）
 * - 输出：
 *   - `orb::referee_status` / `orb::referee_shoot`
 *   - UI：ISR 中仅置位标志，Task 中在安全上下文调用 UI 刷新函数（避免 ISR 直接做耗时工作）
 * - 发送：`Send()` 不直接调用 BSP UART 发送，而是发布 `orb::uart_tx`，由 Drivers/UartTxTask 统一发送。
 *
 * **在线判据/守护**
 * - Online 判据：持续收到新的裁判系统 UART 帧（由 `RxCpltCallback()` 驱动 feed）。
 * - Start() 时会设置 baseline 时间戳，避免“未接入即离线”的误判。
 *
 * **约束**
 * - 启动后不允许重新 Bind（避免竞态与发送到错误串口）。
 * - ISR/回调内必须轻量：只做拷贝、发布 Topic、置位标志与喂狗。
 */

#pragma once
#include "stdint.h"

#include "bsp_uart_port.h"

#include "../communication_topic/uart_topics.hpp"

// Topic pub-sub
#include "../communication_topic/device_topics.hpp"
#include "../communication_topic/topic_pubsub.hpp"

class Referee{
public:
    // Singleton accessor (explicit call-site; no global free-function)
    static Referee& Instance();

    // 初始化并启动（原 Bind + Start 合并）
    void Init(BspUartHandle uart, orb::UartPort tx_port = orb::UartPort::U1);

    void Task();
    void FreshDynamicUI();
    void FreshStaticUI();

    // Optional: send raw bytes via bound UART
    bool Send(uint8_t* data, uint16_t length);

    void RxCpltCallback(uint8_t *buffer, uint16_t length);

    // 只对外暴露需要的字段（从 Topic 拉取最新值；未更新则返回上次缓存）
    uint16_t GetCurrentHp() { (void)status_sub_.copy(status_cache_); return status_cache_.current_hp; }
    uint16_t GetMaxHp() { (void)status_sub_.copy(status_cache_); return status_cache_.max_hp; }
    uint16_t GetChassisPowerLimit() { (void)status_sub_.copy(status_cache_); return status_cache_.chassis_power_limit; }
    bool IsGimbalPowerOn() { (void)status_sub_.copy(status_cache_); return status_cache_.power_management_gimbal_output != 0; }

    uint8_t GetBulletType() { (void)shoot_sub_.copy(shoot_cache_); return shoot_cache_.bullet_type; }
    uint8_t GetLaunchingFrequency() { (void)shoot_sub_.copy(shoot_cache_); return shoot_cache_.launching_frequency; }
    float GetInitialSpeed() { (void)shoot_sub_.copy(shoot_cache_); return shoot_cache_.initial_speed; }

private:
    static void TaskEntry(void *param);

    // ...protocol definitions kept as private nested types to avoid对外暴露完整协议布局...
    struct StatusData {
        uint8_t id;                                     // 本机器人ID
        uint8_t level;                                  // 机器人等级
        uint16_t current_hp;                            // 当前血量
        uint16_t max_hp;                                // 最大血量
        uint16_t shooter_barrel_cooling_value;          // 射击热量每秒冷却值
        uint16_t shooter_barrel_heat_limit;             // 射击热量上限
        uint16_t chassis_power_limit;                   // 底盘功率上限
        // 电源管理模块的输出情况
        // bit0：gimbal口输出，0为无输出，1为24V输出
        // bit1：chassis口输出，0为无输出，1为24V输出
        // bit2：shooter口输出，0为无输出，1为24V输出
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1; 
        uint8_t power_management_shooter_output : 1;
    } __attribute__((packed));
    static constexpr uint16_t kStatusDataId = 0x0201;

    struct ShootData {
        // 弹丸类型
        // 1：17mm 
        // 2：42mm
        uint8_t bullet_type;
        // 发射机构ID
        // 1：17mm发射机构
        // 2：保留位
        // 3：42mm发射机构            
        uint8_t shooter_number;         
        uint8_t launching_frequency;    // 发射频率（单位：Hz）
        float initial_speed;            // 发射初速度（单位：m/s）
    } __attribute__((packed));
    static constexpr uint16_t kShootDataId = 0x0207;

    // 保存解析后的数据（私有）
    StatusData status_{};
    ShootData shoot_{};
    // ISR 中只设置该标志，Task 中处理实际的 UI 更新
    volatile bool ui_update_requested_ = false;

    BspUartHandle uart_ = nullptr;
    orb::UartPort tx_port_ = orb::UartPort::U1;

    osThreadId_t thread_ = nullptr;

    bool started_ = false;

    // Topic 订阅（任务侧/接口侧拉取）
    Sub<orb::RefereeStatus> status_sub_{orb::referee_status};
    Sub<orb::RefereeShoot> shoot_sub_{orb::referee_shoot};
    orb::RefereeStatus status_cache_{};
    orb::RefereeShoot shoot_cache_{};
};