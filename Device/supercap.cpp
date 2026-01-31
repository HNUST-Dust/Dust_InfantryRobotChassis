#include "supercap.h"
#include "cmsis_os.h"
#include <cstdint>

// Topic pub-sub
#include "../communication_topic/device_topics.hpp"

bool Supercap::Bind(BspCanHandle can, uint16_t can_rx_id, uint16_t can_tx_id)
{
    configASSERT(started_ == false);
    if (started_) {
        return false;
    }

    configASSERT(can != nullptr);
    can_handle_ = can;
    can_rx_id_ = can_rx_id;
    can_tx_id_ = can_tx_id;

    // default params
    power_limit_max_ = 55;
    power_compensate_max_ = 50;
    supercap_enable_status_ = SUPERCAP_STATUS_ENABLE;

    // 发送侧：发布 orb::supercap_tx 后自动唤醒发送任务（资源在 Start() 中创建）
    return (can_handle_ != nullptr);
}

bool Supercap::Start()
{
    if (started_) {
        configASSERT(false);
        return false;
    }
    if (can_handle_ == nullptr) {
        configASSERT(false);
        return false;
    }

    // 发送侧：发布 orb::supercap_tx 后自动唤醒发送任务（静态资源，避免动态分配）
    if (tx_event_flags_ == nullptr) {
        tx_evt_attr_ = osEventFlagsAttr_t{
            .name = "supercap_tx_evt",
            .cb_mem = &tx_evt_cb_,
            .cb_size = sizeof(tx_evt_cb_),
        };
        tx_event_flags_ = osEventFlagsNew(&tx_evt_attr_);
        if (!tx_event_flags_) {
            configASSERT(false);
            return false;
        }

        tx_notifier_ = Notifier(tx_event_flags_, kTxEventFlagBit);
        orb::supercap_tx.register_notifier(&tx_notifier_);

        const osThreadAttr_t kSupercapTxTaskAttr = {
            .name = "supercap_tx",
            .cb_mem = &tx_tcb_,
            .cb_size = sizeof(tx_tcb_),
            .stack_mem = tx_stack_,
            .stack_size = sizeof(tx_stack_),
            .priority = (osPriority_t)osPriorityNormal,
        };
        tx_thread_ = osThreadNew(Supercap::TxTaskEntry, this, &kSupercapTxTaskAttr);
        if (!tx_thread_) {
            configASSERT(false);
            return false;
        }
    }

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
        return false;
    }

    started_ = true;
    return true;
}

/**
 * @brief 超级电容初始化
 * 
 * @param hcan 
 * @param can_rx_id 
 * @param can_tx_id 
 */
void Supercap::Init(
    BspCanHandle can,
    uint16_t can_rx_id,
    uint16_t can_tx_id)
{
    (void)Bind(can, can_rx_id, can_tx_id);
    (void)Start();
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
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

    // sliding window / alive
    flag_ += 1;

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

void Supercap::AlivePeriodElapsedCallback()
{
    // TODO:待实现
}

/**
 * @brief 超级电容数据处理
 * 
 */
void Supercap::DataProcess()
{
    // migrated: parsing is done in CanRxCpltCallback(const BspCanFrame*)
}

/**
 * @brief 超级电容CAN通讯发送回调函数
 * 
 */
void Supercap::SendPeriodElapsedCallback()
{
    // legacy: keep periodic API but route through Topic queue
    orb::SupercapTx tx{};
    tx.supercap_enable_status = static_cast<uint8_t>(supercap_enable_status_);

    // legacy SupercapStatus -> topic strong enum
    tx.supercap_charge_status = (supercap_charge_status_ == SUPERCAP_STATUS_CHARGE)
                                   ? orb::SupercapChargeMode::Charge
                                   : orb::SupercapChargeMode::Discharge;

    tx.power_limit_max = power_limit_max_;
    tx.charge_power = charge_power_;
    orb::supercap_tx.publish(tx);
}

void Supercap::TxTaskEntry(void *argument)
{
    auto *self = static_cast<Supercap *>(argument);
    self->TxTask();
}

void Supercap::TxTask()
{
    RingSub<orb::SupercapTx, 4> sub(orb::supercap_tx);

    for (;;) {
        (void)osEventFlagsWait(tx_event_flags_, kTxEventFlagBit, osFlagsWaitAny, osWaitForever);

        orb::SupercapTx msg{};
        while (sub.copy(msg)) {
            if (can_handle_ == nullptr) {
                continue;
            }

            BspCanFrame frame{};
            frame.id = can_tx_id_;
            frame.id_type = BSP_CAN_ID_STD;
            frame.frame_type = BSP_CAN_FRAME_DATA;
            frame.is_fd = false;
            frame.brs = false;
            frame.len = 8;

            frame.data[0] = msg.supercap_enable_status;
            frame.data[1] = static_cast<uint8_t>(msg.supercap_charge_status);
            frame.data[2] = msg.power_limit_max;
            frame.data[3] = msg.charge_power;
            frame.data[4] = 0;
            frame.data[5] = 0;
            frame.data[6] = 0;
            frame.data[7] = 0;

            (void)bsp_can_send(can_handle_, &frame);
        }
    }
}

void Supercap::Task()
{
    for (;;)
    {
        AlivePeriodElapsedCallback();
        // 这里仍保留周期发送：内部转为 publish -> 自动发送
        SendPeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(10));
    }
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
