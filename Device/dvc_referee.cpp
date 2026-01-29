#include "dvc_referee.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include <cstdint>
#include <cstring>
void Referee::Init()
{
    static const osThreadAttr_t kRefereeTaskAttr = {
        .name = "referee_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    osThreadNew(Referee::TaskEntry, this, &kRefereeTaskAttr);
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
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        std::memcpy(&status_, payload, sizeof(StatusData));
        ui_update_requested_ = true; 
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    } else if (msg_id == kShootDataId && payload_len >= sizeof(ShootData)) {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        std::memcpy(&shoot_, payload, sizeof(ShootData));
        ui_update_requested_ = true;
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
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
            FreshDynamicUI();
        }

        // ui_booster_off_now_strings->color = 0;
        ui_update_booster_off();
        // 内部发送函数后已有10ms延时
        osDelay(pdMS_TO_TICKS(10));
    }

}
