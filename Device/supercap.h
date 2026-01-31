#ifndef DEVICE_SUPERCAP_H_
#define DEVICE_SUPERCAP_H_

#include <cstdint>

// Platform (HAL-free) CAN abstraction
#include "bsp_can_port.h"

#include "cmsis_os2.h"

extern "C" {
#include "FreeRTOS.h"
#include "event_groups.h"
}

// Topic notify
#include "../communication_topic/topic_notify.hpp"

/**
 * @brief 超级电容状态
 * @note  包含工作状态和充放电状态
 */
enum SupercapStatus
{
    SUPERCAP_STATUS_DISABLE = 0,
    SUPERCAP_STATUS_ENABLE = 1,
    SUPERCAP_STATUS_CHARGE = 1,
    SUPERCAP_STATUS_DISCHARGE = 0,
};

/**
 * @brief 超级电容状态码
 * 
 */
enum SupercapStatusCode
{
    DISCHARGE = 0,              // 放电
    CHARGE = 1,                 // 充电
    WAIT = 2,                   // 待机 
    SOFRSTART_PROTECTION = 3,   // 软启动保护
    OCP_PROTECTION = 4,         // 过流保护
    OVP_BAT_PROTECTION = 5,     // 电池过压保护
    UVP_BAT_PROTECTION = 6,     // 电池欠压保护
    UVP_CAP_PROTECTION = 7,     // 电容欠压保护
    OTP_PROTECTION = 8          // 过温保护
};

/**
 * @brief 超级电容接收数据
 * @note 200Hz频率，可调
 */
struct SupercapRecivedData
{
    SupercapStatus supercap_work_status;        // 超级电容可用状态
    SupercapStatusCode supercap_status_code;    // 超级电容充放电状态
    uint8_t supercap_energy_percent;            // 超级电容剩余能量百分比 0~100% 0%的时候会自动关闭
    uint8_t chassis_compensate_power;           // 超级电容当前功率消耗 W 范围：0~255
    uint8_t battery_voltage;                    // 电池电压 V，放大了10倍，
};

/**
 * @brief 超级电容发送数据
 * 
 */
struct SupercapSendData
{
    SupercapStatus supercap_enable_status;        // 超级电容工作状态
    SupercapStatus supercap_charge_status;        // 超级电容充放电状态
    uint8_t power_limit_max;                      // 底盘功率限制 W
    uint8_t charge_power;                         // 充电功率 W
};

class Supercap
{
public:
    // 依赖注入（HAL-free）
    bool Bind(
        BspCanHandle can,
        uint16_t can_rx_id = 0x100,
        uint16_t can_tx_id = 0x001
    );

    // 启动任务（只做 RTOS 资源创建）
    bool Start();

    // Backward-compatible init (HAL-free)
    void Init(
        BspCanHandle can,
        uint16_t can_rx_id = 0x100,
        uint16_t can_tx_id = 0x001
    );

    inline SupercapStatus GetWorkStatus();
    
    inline SupercapStatusCode GetStatusCode();
    
    inline uint8_t GetEnergyPercent();

    inline uint8_t GetChassisCompensatePower();

    inline uint8_t GetBatteryVoltage();

    // Platform RX (HAL-free): feed a decoded CAN frame
    void CanRxCpltCallback(const BspCanFrame *frame);

    void AlivePeriodElapsedCallback();

    // legacy: keep periodic hook (internally publishes Topic)
    void SendPeriodElapsedCallback();

    void Task();
protected:
    // Platform CAN handle（preferred）
    BspCanHandle can_handle_ = nullptr;

    uint16_t can_rx_id_ = 0x100;
    uint16_t can_tx_id_ = 0x003;
    SupercapRecivedData recived_data_ = {};
    uint32_t flag_ = 0;
    uint32_t pre_flag_ = 0;

    // ——以下为内部状态（不再对外提供 SetXXX 接口，必须通过 Topic 控制）——
    SupercapStatus supercap_enable_status_ = SUPERCAP_STATUS_DISABLE;
    SupercapStatus supercap_work_status_ = SUPERCAP_STATUS_DISABLE;
    SupercapStatus supercap_charge_status_ = SUPERCAP_STATUS_CHARGE;
    SupercapStatusCode supercap_status_code_ = {};
    uint8_t power_limit_max_ = 0;
    uint8_t charge_power_ = 0;
    uint8_t supercap_energy_percent_ = 0;
    uint8_t chassis_power_ = 0;
    uint8_t battery_voltage_ = 0;
    uint8_t power_compensate_max_ = 150.0f;

    void DataProcess();
    void Output();
    static void TaskEntry(void *param);

    osThreadId_t thread_ = nullptr;

    StaticTask_t tcb_{};
    StackType_t stack_[512]{};

    // ===== Topic 化发送：发布即自动发送 =====
    static void TxTaskEntry(void *param);
    void TxTask();

    osThreadId_t tx_thread_ = nullptr;

    StaticTask_t tx_tcb_{};
    StackType_t tx_stack_[384]{};

    osEventFlagsId_t tx_event_flags_ = nullptr;
    StaticEventGroup_t tx_evt_cb_{};
    osEventFlagsAttr_t tx_evt_attr_{};

    static constexpr uint32_t kTxEventFlagBit = 1u << 0;
    Notifier tx_notifier_{nullptr, kTxEventFlagBit};

    bool started_ = false;

};

/**
 * @brief 获取超级电容在线状态
 *
 * @return uint8_t 超级电容在线状态
 */
inline SupercapStatus Supercap::GetWorkStatus()
{
    return (recived_data_.supercap_work_status);
}

/**
 * @brief 获取超级电容状态码
 * 
 * @return SupercapStatusCode 
 */
inline SupercapStatusCode Supercap::GetStatusCode()
{
    return (recived_data_.supercap_status_code);
}

/**
 * @brief 获取超级电容剩余能量百分比
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetEnergyPercent()
{
    return (recived_data_.supercap_energy_percent);
}

/**
 * @brief 获取底盘消耗功率
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetChassisCompensatePower()
{
    return (recived_data_.chassis_compensate_power);
}

/**
 * @brief 获取电池电压
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetBatteryVoltage()
{
    return (recived_data_.battery_voltage);
}

#endif // !DEVICE_SUPERCAP_H_
