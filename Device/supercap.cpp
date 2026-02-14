/**
 * @file supercap.cpp
 * @brief Supercap 实现：周期发送 + CAN RX 回调解析。
 *
 * 约定：
 * - RX：由 Platform/CAN 回调喂入 `CanRxCpltCallback()`，解析后发布 `orb::supercap_rx`。
 * - TX：周期任务打包 CAN 帧并发布 `orb::can_tx`。
 * - 实际 CAN 发送由 Drivers/CanTxTask 统一落地。
 */

#include "supercap.h"
#include <cstdint>

#include "../communication_topic/can_topics.hpp"

// Topic pub-sub
#include "../communication_topic/device_topics.hpp"
#include "../communication_topic/mcu_topics.hpp"

Supercap& Supercap::Instance()
{
    static Supercap inst;
    return inst;
}

/**
 * @brief 超级电容初始化
 * 
 * @param hcan 
 * @param can_rx_id 
 * @param can_tx_id 
 */
void Supercap::Init(
    orb::CanBus bus,
    BspCanHandle can,
    uint16_t can_rx_id,
    uint16_t can_tx_id)
{
    configASSERT(started_ == false);
    if (started_) {
        return;
    }

    configASSERT(can != nullptr);
    if (can == nullptr) {
        return;
    }

    can_handle_ = can;
    tx_bus_ = bus;
    can_rx_id_ = can_rx_id;
    can_tx_id_ = can_tx_id;

    // default params
    power_limit_max_ = 55;
    power_compensate_max_ = 50;
    supercap_enable_status_ = SUPERCAP_STATUS_ENABLE;

    static const osThreadAttr_t kSupercapTaskAttr = {
        .name = "supercap_task",
        .cb_mem = &tcb_,
        .cb_size = sizeof(tcb_),
        .stack_mem = stack_,
        .stack_size = sizeof(stack_),
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Supercap::TaskEntry, this, &kSupercapTaskAttr);
    if (!thread_) {
        configASSERT(false);
        return;
    }

    started_ = true;
}

void Supercap::TaskEntry(void *argument)
{
    Supercap *self = static_cast<Supercap *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief 超级电容CAN通讯接收回调函数
 * 
 * @param rx_data 
 */
void Supercap::CanRxCpltCallback(const BspCanFrame *frame)
{
    if (frame == nullptr) {
        return;
    }

    // Filter by ID if configured
    if (can_rx_id_ != 0 && frame->id != can_rx_id_) {
        return;
    }

    if (frame->len < sizeof(SupercapRecivedData)) {
        return;
    }

    const auto *temp_buffer = reinterpret_cast<const SupercapRecivedData *>(frame->data);
    recived_data_.supercap_work_status = temp_buffer->supercap_work_status;
    recived_data_.supercap_status_code = temp_buffer->supercap_status_code;
    recived_data_.supercap_energy_percent = temp_buffer->supercap_energy_percent;
    recived_data_.chassis_compensate_power = temp_buffer->chassis_compensate_power;
    recived_data_.battery_voltage = temp_buffer->battery_voltage;

    // Topic 发布：让业务层通过订阅获取，不再直接探测 supercap_ 内部状态
    orb::SupercapRx msg{};
    msg.supercap_work_status = static_cast<uint8_t>(recived_data_.supercap_work_status);
    msg.supercap_status_code = static_cast<uint8_t>(recived_data_.supercap_status_code);
    msg.supercap_energy_percent = recived_data_.supercap_energy_percent;
    msg.chassis_compensate_power = recived_data_.chassis_compensate_power;
    msg.battery_voltage = recived_data_.battery_voltage;
    orb::supercap_rx.publish(msg);
}


/**
 * @brief 超级电容CAN通讯发送回调函数
 * 
 */
void Supercap::SendPeriodElapsedCallback()
{
    // legacy: keep periodic API but send CAN frame directly
    orb::CanTxFrame frame{};
    frame.bus = tx_bus_;
    frame.id = can_tx_id_;
    frame.id_type = orb::CanIdType::Std;
    frame.frame_type = orb::CanFrameType::Data;
    frame.is_fd = false;
    frame.brs = false;
    frame.len = 8;

    frame.data[0] = static_cast<uint8_t>(supercap_enable_status_);
    frame.data[1] = (supercap_charge_status_ == SUPERCAP_STATUS_CHARGE)
                        ? static_cast<uint8_t>(orb::SupercapChargeMode::Charge)
                        : static_cast<uint8_t>(orb::SupercapChargeMode::Discharge);
    frame.data[2] = power_limit_max_;
    frame.data[3] = charge_power_;

    orb::can_tx.publish(frame);
}

void Supercap::Task()
{
    // Periodic-only sending: build TX command each cycle
    Subscription<orb::McuControl> mcu_control_sub(orb::mcu_control);
    orb::McuControl mcu{};

    for (;;)
    {
        (void)mcu_control_sub.copy(mcu);

        // keep enabled by default
        supercap_enable_status_ = SUPERCAP_STATUS_ENABLE;

        supercap_charge_status_ = (mcu.supercap == orb::SupercapUserCmd::Charge)
                                     ? SUPERCAP_STATUS_CHARGE
                                     : SUPERCAP_STATUS_DISCHARGE;

        // legacy fixed params
        power_limit_max_ = 100;
        charge_power_ = 50;

        // periodic CAN send
        SendPeriodElapsedCallback();
        osDelay(10);
    }
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
