/**
 * @file dvc_MCU_comm.h
 * @brief 外部 MCU 通信（CAN RX 解包 → Topic 发布；Topic 订阅 → CAN TX 发布）
 *
 * 设计思路：
 * =========
 * `McuComm` 将“外部 MCU/上位机”的数据输入统一转换为 Topic，供业务模块订阅。
 * 同时它也承担一条典型的发送链路：订阅待发送 RingTopic，组帧后发布到 `orb::can_tx`。
 *
 * 关键原则：
 * =========
 * - RX：回调快速解析并发布 Topic。
 * - TX：不直接调用 BSP 发送；只发布 `orb::can_tx`，由统一 TxTask 完成最终发送。
 * - 守护：在线判据基于“收到新外部数据”而非任务空转。
 *
 * 线程模型：
 * =========
 * - RX 回调：由平台 CAN 接收触发（可能在 IRQ 或专用接收线程）。
 * - TxTask：CMSIS-RTOS2 线程，等待 notifier 唤醒后批量发送。
 */
#ifndef MODULES_COMM_DVC_MCU_COMM_H
#define MODULES_COMM_DVC_MCU_COMM_H

#include <cstdint>
#include <cstring>

// Platform (HAL-free) CAN abstraction
#include "bsp_can_port.h"

#include "../communication_topic/can_topics.hpp"

// Topic pub-sub
#include "../communication_topic/topic_pubsub.hpp"
#include "../communication_topic/mcu_topics.hpp"
#include "cmsis_os2.h"

extern "C" {
#include "FreeRTOS.h"
#include "event_groups.h"
}

// legacy ChassisSpinMode 已迁移到 orb::McuChassisSpinMode

struct McuCommData
{
    uint8_t                 yaw              = 127;                     // yaw
    uint8_t                 pitch_angle      = 127;                     // 俯仰角度
    uint8_t                 chassis_speed_x  = 127;                     // 平移方向：前、后、左、右
    uint8_t                 chassis_speed_y  = 127;                     // 底盘移动总速度
    uint8_t                 chassis_rotation = 127;                     // 自转：不转、顺时针转、逆时针转
    orb::McuChassisSpinMode chassis_spin     = orb::McuChassisSpinMode::Disable;
    orb::SupercapUserCmd    supercap         = orb::SupercapUserCmd::Charge;
    uint8_t                 auto_aim_flag    = 0;                       // 自瞄标志
    uint8_t                 reset_zero       = 0;                       // 云台设置零点
};

constexpr uint8_t REMOTE_CONTROL_ID = 0xAB;

struct McuAutoaimData
{
    float yaw_angle;
    float yaw_omega;
    float yaw_torque;
    float pitch_angle;
    float pitch_omega;
    float pitch_torque;
};
constexpr uint8_t AUTOAIM_INFO_ID    = 0xFA;

struct McuImuData
{
    float yaw_total_angle_f;
    float pitch_f;
    float yaw_omega_f;
    float pitch_omega_f;
};
constexpr uint8_t IMU_INFO_ID    = 0xAE;

class McuComm
{
public:

    // Singleton accessor (explicit call-site; no global free-function)
    static McuComm& Instance();

    volatile McuCommData mcu_comm_data_ = {
            127,
            127,
            127,
            127,
            127,
            orb::McuChassisSpinMode::Disable,
            orb::SupercapUserCmd::Charge,
            0,
            0,
    };

    McuAutoaimData mcu_autoaim_data_ = {
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
    };

    McuImuData mcu_imu_data_ = {
            0,
            0,
            0,
            0,
    };

    // 初始化并启动（原 Bind + Start 合并）
    void Init(
        orb::CanBus bus,
        BspCanHandle can,
        uint8_t can_rx_id,
        uint8_t can_tx_id
    );

    // RX callbacks (Platform frame)
    void CanRemoteControlRxCpltCallback(const BspCanFrame* frame);
    void CanAutoAimInfoRxCpltCallback(const BspCanFrame* frame);
    void CanImuInfoRxCpltCallback(const BspCanFrame* frame);

    void Task();

protected:
    // Platform CAN handle
    BspCanHandle can_handle_ = nullptr;

    // TX publish target bus (unified CanTxTask)
    orb::CanBus tx_bus_ = orb::CanBus::CAN1;

    // 收数据绑定的CAN ID
    uint16_t can_rx_id_;
    // 发数据绑定的CAN ID
    uint16_t can_tx_id_;

    // 发送缓冲区（GIMBAL_INFO_ID 使用 16B FD 帧）
    uint8_t tx_data_[16] = {0};

    // 发布到 Topic（让消费者不再直接读 mcu_comm_ 内部成员）
    Publisher<orb::McuControl> mcu_control_pub_{orb::mcu_control};
    Publisher<orb::McuAutoAim> mcu_autoaim_pub_{orb::mcu_autoaim};
    Publisher<orb::McuImu> mcu_imu_pub_{orb::mcu_imu};

    // 从 Topic 订阅待发送数据（RingTopic）
    RingSub<orb::GimbalInfoTx, 4> gimbal_info_tx_sub_{orb::gimbal_info_tx};

    // 自动发送通知
    osThreadId_t tx_thread_ = nullptr;

    osEventFlagsId_t tx_evt_ = nullptr;
    StaticEventGroup_t tx_evt_cb_{};
    osEventFlagsAttr_t tx_evt_attr_{};

    static constexpr uint32_t kTxEvtMask = 0x01;
    Notifier tx_notifier_{nullptr, kTxEvtMask};

    bool started_ = false;

    bool auto_tx_started_ = false;

    static void TxTaskEntry(void *param);
    void TxTask();

    // 内部函数
    void DataProcess();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //MODULES_COMM_DVC_MCU_COMM_H
