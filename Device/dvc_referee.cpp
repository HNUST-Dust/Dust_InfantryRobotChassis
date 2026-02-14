/**
 * @file dvc_referee.cpp
 * @brief Referee 实现：UART 帧解析、Topic 发布、以及 UI 更新的任务侧调度。
 *
 * 说明：
 * - RX 回调按“msg_id + payload”形式解析（仅覆盖本工程实际使用的数据类型）。
 * - 解析出的状态通过 Topic 发布给业务层，避免业务层依赖裁判协议细节。
 * - Send() 通过 `orb::uart_tx` 统一出口发送，避免直接调用 BSP。
 * - daemon_supervisor feed：由 RX 回调在收到有效帧时触发。
 */

#include "dvc_referee.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include <cstdint>
#include <cstring>
#include "ui.h"
// Topic pub-sub
#include "../communication_topic/device_topics.hpp"

#include "../communication_topic/uart_topics.hpp"


Referee& Referee::Instance()
{
    static Referee inst;
    return inst;
}

void Referee::Init(BspUartHandle uart, orb::UartPort tx_port)
{
    // 严格策略：已启动后不允许重新绑定，避免竞态/发送到错误串口。
    configASSERT(started_ == false);
    if (started_) {
        return;
    }

    configASSERT(uart != nullptr);
    if (uart == nullptr) {
        return;
    }

    uart_ = uart;
    tx_port_ = tx_port;

    static const osThreadAttr_t kRefereeTaskAttr = {
        .name = "referee_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    thread_ = osThreadNew(Referee::TaskEntry, this, &kRefereeTaskAttr);
    if (!thread_) {
        configASSERT(false);
        return;
    }

    started_ = true;
}

bool Referee::Send(uint8_t* data, uint16_t length)
{
    if (uart_ == nullptr) {
        return false;
    }

    if (data == nullptr || length == 0) {
        return false;
    }
    if (length > 256) {
        return false;
    }

    orb::UartTxFrame pkt{};
    pkt.port = tx_port_;
    pkt.len = length;
    std::memcpy(pkt.bytes, data, length);
    pkt.throttle_ms = 0;
    orb::uart_tx.publish(pkt);
    return true;
}

void Referee::TaskEntry(void *param)
{
    Referee *self = static_cast<Referee *>(param);
    self->Task();
}

void Referee::RxCpltCallback(uint8_t *buffer, uint16_t length)
{
    // 协议假定：前 2 字节为消息 ID（little-endian），后面为 payload
    if (buffer == nullptr || length < 2) {
        return;
    }

    uint16_t msg_id = static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
    uint8_t *payload = buffer + 2;
    uint16_t payload_len = (length > 2) ? (length - 2) : 0;

    if (msg_id == kStatusDataId && payload_len >= sizeof(StatusData)) {
        StatusData s{};
        std::memcpy(&s, payload, sizeof(StatusData));

        // Topic 发布（业务层订阅读取）
        orb::RefereeStatus out{};
        out.id = s.id;
        out.level = s.level;
        out.current_hp = s.current_hp;
        out.max_hp = s.max_hp;
        out.shooter_barrel_cooling_value = s.shooter_barrel_cooling_value;
        out.shooter_barrel_heat_limit = s.shooter_barrel_heat_limit;
        out.chassis_power_limit = s.chassis_power_limit;
        out.power_management_gimbal_output = s.power_management_gimbal_output;
        out.power_management_chassis_output = s.power_management_chassis_output;
        out.power_management_shooter_output = s.power_management_shooter_output;
        orb::referee_status.publish(out);

        ui_update_requested_ = true;
    } else if (msg_id == kShootDataId && payload_len >= sizeof(ShootData)) {
        ShootData sh{};
        std::memcpy(&sh, payload, sizeof(ShootData));

        orb::RefereeShoot out{};
        out.bullet_type = sh.bullet_type;
        out.shooter_number = sh.shooter_number;
        out.launching_frequency = sh.launching_frequency;
        out.initial_speed = sh.initial_speed;
        orb::referee_shoot.publish(out);

        ui_update_requested_ = true;
    }
}

void Referee::Task()
{
    ui_init_booster_off();
    for(;;)
    {
        // 任务上下文检查 ISR 设置的标志并在安全上下文调用 UI 更新
        bool do_update = false;
        taskENTER_CRITICAL();
        if (ui_update_requested_) {
            ui_update_requested_ = false;
            do_update = true;
        }
        taskEXIT_CRITICAL();

        if (do_update) {
            // FreshDynamicUI();
        }

        // ui_booster_off_now_strings->color = 0;
        ui_update_booster_off();
        // 内部发送函数后已有10ms延时
        osDelay(pdMS_TO_TICKS(10));
    }

}
