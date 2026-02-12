/**
 * @file dvc_MCU_comm.cpp
 * @brief McuComm 实现：CAN RX 解包、Topic 发布、TxTask 组帧并发布 can_tx
 *
 * 核心逻辑：
 * =========
 * - `Start()`：注册 daemon client（COMM/CRITICAL/FATAL），并启动 TxTask。
 * - `Can*RxCpltCallback()`：
 *   - 校验帧有效性
 *   - 收到新数据时 feed daemon（在线判据：有新外部数据）
 *   - 解析字段并发布到对应 Topic
 * - `TxTask()`：
 *   - 等待 `orb::gimbal_info_tx` notifier 唤醒
 *   - 批量读取 RingTopic 并将数据转换为 `orb::CanTxFrame`
 *   - 发布到 `orb::can_tx`
 *
 * 为什么 TX 不直接发 CAN：
 * =====================
 * 工程约束为“CAN/UART 发送必须经由唯一 TxTask 出口”。
 * `McuComm` 作为上游，只负责把待发送的数据发布到 Topic。
 */

#include "dvc_MCU_comm.h"
#include <cstdint>
#include <cstring>

#include "../communication_topic/can_topics.hpp"

#include "bsp_dwt.h"
#include "../daemon_supervisor/supervisor.hpp"

namespace {
inline uint32_t now_ms()
{
    return static_cast<uint32_t>(dwt_get_timeline_ms());
}

void mcu_comm_daemon_fault(DaemonClient&) {}

DaemonClient* s_mcu_comm_daemon = nullptr;
} // namespace

McuComm& McuComm::Instance()
{
    static McuComm inst;
    return inst;
}

bool McuComm::Bind(orb::CanBus bus, BspCanHandle can, uint8_t can_rx_id, uint8_t can_tx_id)
{
    configASSERT(started_ == false);
    if (started_) {
        return false;
    }

    configASSERT(can != nullptr);
    can_handle_ = can;
    tx_bus_ = bus;
    can_rx_id_ = can_rx_id;
    can_tx_id_ = can_tx_id;
    return (can_handle_ != nullptr);
}

bool McuComm::Start()
{
    // 当前 McuComm 的“启动”语义就是启动自动发送 TxTask（原 StartAutoTx）
    if (started_) {
        configASSERT(false);
        return false;
    }
    if (can_handle_ == nullptr) {
        configASSERT(false);
        return false;
    }

    // Online criterion: receiving fresh CAN data from external MCU.
    {
        static DaemonClient daemon(
            200,
            mcu_comm_daemon_fault,
            this,
            DaemonClient::Domain::COMM,
            DaemonClient::FaultLevel::FATAL,
            DaemonClient::Priority::CRITICAL);
        s_mcu_comm_daemon = &daemon;
        (void)DaemonSupervisor::register_client(s_mcu_comm_daemon);
        // Baseline timestamp; subsequent feed is driven by actual RX callbacks.
        s_mcu_comm_daemon->feed(now_ms());
    }

    StartAutoTx();
    started_ = true;
    return true;
}

void McuComm::Init(orb::CanBus bus, BspCanHandle can, uint8_t can_rx_id, uint8_t can_tx_id)
{
    (void)Bind(bus, can, can_rx_id, can_tx_id);
    // 兼容旧逻辑：Init 后仍由外部调用 StartAutoTx/Start；这里不强制调用 Start()
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void McuComm::TaskEntry(void *argument) {
     McuComm *self = static_cast<McuComm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

// 实际任务逻辑
void McuComm::Task() {
     struct McuCommData mcu_comm_data_local;
     for (;;)
     {
          // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // mcu_comm_data__Local = *const_cast<const struct McuCommData*>(&(mcu_comm_data_));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}


void McuComm::StartAutoTx()
{
    if (auto_tx_started_) {
        configASSERT(false);
        return;
    }
    auto_tx_started_ = true;

    // 创建事件标志并绑定 notifier（静态内存，避免动态分配）
    tx_evt_attr_ = osEventFlagsAttr_t{
        .name = "mcu_tx_evt",
        .cb_mem = &tx_evt_cb_,
        .cb_size = sizeof(tx_evt_cb_),
    };
    tx_evt_ = osEventFlagsNew(&tx_evt_attr_);
    if (!tx_evt_) {
        configASSERT(false);
        return;
    }

    tx_notifier_ = Notifier(tx_evt_, kTxEvtMask);
    orb::gimbal_info_tx.register_notifier(&tx_notifier_);

    static const osThreadAttr_t kMcuCommTxTaskAttr = {
        .name = "mcu_tx_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal,
    };
    tx_thread_ = osThreadNew(McuComm::TxTaskEntry, this, &kMcuCommTxTaskAttr);
    if (!tx_thread_) {
        configASSERT(false);
        return;
    }
}

void McuComm::TxTaskEntry(void *param)
{
    auto *self = static_cast<McuComm*>(param);
    self->TxTask();
}

void McuComm::TxTask()
{
    for (;;)
    {
        // 等待 Topic 发布唤醒
        (void)osEventFlagsWait(tx_evt_, kTxEvtMask, osFlagsWaitAny, osWaitForever);

        // 读出并发送所有待发数据
        orb::GimbalInfoTx tx_msg{};
        while (gimbal_info_tx_sub_.copy(tx_msg))
        {
            uint8_t can_tx_frame[16];
            union { float f; uint8_t b[4]; } conv;

            conv.f = tx_msg.yaw_angle;
            can_tx_frame[0] = conv.b[0];
            can_tx_frame[1] = conv.b[1];
            can_tx_frame[2] = conv.b[2];
            can_tx_frame[3] = conv.b[3];

            conv.f = tx_msg.yaw_omega;
            can_tx_frame[4] = conv.b[0];
            can_tx_frame[5] = conv.b[1];
            can_tx_frame[6] = conv.b[2];
            can_tx_frame[7] = conv.b[3];

            conv.f = tx_msg.pitch_angle;
            can_tx_frame[8] = conv.b[0];
            can_tx_frame[9] = conv.b[1];
            can_tx_frame[10] = conv.b[2];
            can_tx_frame[11] = conv.b[3];

            conv.f = tx_msg.pitch_omega;
            can_tx_frame[12] = conv.b[0];
            can_tx_frame[13] = conv.b[1];
            can_tx_frame[14] = conv.b[2];
            can_tx_frame[15] = conv.b[3];

            orb::CanTxFrame tx{};
            tx.bus = tx_bus_;
            tx.id = orb::GIMBAL_INFO_ID;
            tx.id_type = orb::CanIdType::Std;
            tx.frame_type = orb::CanFrameType::Data;
            tx.is_fd = true;
            tx.brs = true;
            tx.len = 16;
            std::memset(tx.data, 0, sizeof(tx.data));
            std::memcpy(tx.data, can_tx_frame, 16);
            orb::can_tx.publish(tx);
        }
    }
}

static inline const uint8_t* frame_payload_or_null(const BspCanFrame* frame, uint8_t min_len)
{
    if (frame == nullptr) return nullptr;
    if (frame->frame_type != BSP_CAN_FRAME_DATA) return nullptr;
    if (frame->len < min_len) return nullptr;
    return frame->data;
}

void McuComm::CanRemoteControlRxCpltCallback(const BspCanFrame* frame)
{
    const uint8_t* rx_data = frame_payload_or_null(frame, 9);
    if (rx_data == nullptr) {
        return;
    }

    if (s_mcu_comm_daemon) {
        s_mcu_comm_daemon->feed(now_ms());
    }

    mcu_comm_data_.yaw             = rx_data[0];
    mcu_comm_data_.pitch_angle     = rx_data[1];
    mcu_comm_data_.chassis_speed_x = rx_data[2];
    mcu_comm_data_.chassis_speed_y = rx_data[3];
    mcu_comm_data_.chassis_rotation = rx_data[4];

    // 强类型：内部缓存也使用 orb::McuChassisSpinMode
    switch (rx_data[5])
    {
        case 0: mcu_comm_data_.chassis_spin = orb::McuChassisSpinMode::Clockwise; break;
        case 1: mcu_comm_data_.chassis_spin = orb::McuChassisSpinMode::Disable; break;
        case 2: mcu_comm_data_.chassis_spin = orb::McuChassisSpinMode::CounterClockwise; break;
        default: mcu_comm_data_.chassis_spin = orb::McuChassisSpinMode::Disable; break;
    }

    // 强类型：内部缓存也使用 orb::SupercapUserCmd
    switch (rx_data[6])
    {
        case 0: mcu_comm_data_.supercap = orb::SupercapUserCmd::Charge; break;
        case 1: mcu_comm_data_.supercap = orb::SupercapUserCmd::Discharge; break;
        default: mcu_comm_data_.supercap = orb::SupercapUserCmd::Charge; break;
    }

    mcu_comm_data_.auto_aim_flag = rx_data[7];
    mcu_comm_data_.reset_zero    = rx_data[8];

    // Publish to topic
    orb::McuControl msg{};
    msg.yaw = mcu_comm_data_.yaw;
    msg.pitch_angle = mcu_comm_data_.pitch_angle;
    msg.chassis_speed_x = mcu_comm_data_.chassis_speed_x;
    msg.chassis_speed_y = mcu_comm_data_.chassis_speed_y;
    msg.chassis_rotation = mcu_comm_data_.chassis_rotation;
    msg.chassis_spin = mcu_comm_data_.chassis_spin;
    msg.supercap = mcu_comm_data_.supercap;
    msg.auto_aim_flag = mcu_comm_data_.auto_aim_flag;
    msg.reset_zero = mcu_comm_data_.reset_zero;
    mcu_control_pub_.publish(msg);
}

void McuComm::CanAutoAimInfoRxCpltCallback(const BspCanFrame* frame)
{
    const uint8_t* rx_data = frame_payload_or_null(frame, 8);
    if (rx_data == nullptr) {
        return;
    }

    if (s_mcu_comm_daemon) {
        s_mcu_comm_daemon->feed(now_ms());
    }

    memcpy(&mcu_autoaim_data_.yaw_angle, &rx_data[0], 4);
    memcpy(&mcu_autoaim_data_.pitch_angle, &rx_data[4], 4);

    // Publish to topic
    orb::McuAutoAim msg{};
    msg.yaw_angle = mcu_autoaim_data_.yaw_angle;
    msg.pitch_angle = mcu_autoaim_data_.pitch_angle;
    mcu_autoaim_pub_.publish(msg);
}

void McuComm::CanImuInfoRxCpltCallback(const BspCanFrame* frame)
{
    const uint8_t* rx_data = frame_payload_or_null(frame, 16);
    if (rx_data == nullptr) {
        return;
    }

    if (s_mcu_comm_daemon) {
        s_mcu_comm_daemon->feed(now_ms());
    }

    memcpy(&mcu_imu_data_.yaw_total_angle_f, &rx_data[0], 4);
    memcpy(&mcu_imu_data_.pitch_f, &rx_data[4], 4);
    memcpy(&mcu_imu_data_.yaw_omega_f, &rx_data[8], 4);
    memcpy(&mcu_imu_data_.pitch_omega_f, &rx_data[12], 4);

    // Publish to topic
    orb::McuImu msg{};
    msg.yaw_total_angle_f = mcu_imu_data_.yaw_total_angle_f;
    msg.pitch_f = mcu_imu_data_.pitch_f;
    msg.yaw_omega_f = mcu_imu_data_.yaw_omega_f;
    msg.pitch_omega_f = mcu_imu_data_.pitch_omega_f;
    mcu_imu_pub_.publish(msg);
}
