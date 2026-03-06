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
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    } else if (msg_id == kShootDataId && payload_len >= sizeof(ShootData)) {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        std::memcpy(&shoot_, payload, sizeof(ShootData));
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    } else if (msg_id == kGameStatusId && payload_len >= sizeof(GameStatus)) {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        std::memcpy(&game_status_, payload, sizeof(GameStatus));
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }
}

void Referee::Task()
{
    ui_init_booster_off();

    for (;;) {
        FreshDynamicUI();
        ui_update_booster_off();

        osDelay(pdMS_TO_TICKS(50));
    }
}

void Referee::FreshDynamicUI()
{
    // 从裁判系统状态包同步本机器人 ID
    if (status_.id != 0) {
        ui_self_id = status_.id;
        // ui_self_id = 1; // 目前先写死为1，后续根据实际情况修改

    }

    // booster 状态文字：booster_on / booster_off
    const char* booster_str = booster_status_ ? "booster_on" : "booster_off";
    strcpy(ui_booster_off_dynamic_group_booster_off_text->string, booster_str);
    ui_booster_off_dynamic_group_booster_off_text->str_length = strlen(booster_str);
    ui_booster_off_dynamic_group_booster_off_text->color = booster_status_ ? 2 : 5;

    // 小陀螺状态文字：spin_on / spin_off
    const char* spin_str = spin_status_ ? "spin_on" : "spin_off";
    strcpy(ui_booster_off_dynamic__group_spin_off_text->string, spin_str);
    ui_booster_off_dynamic__group_spin_off_text->str_length = strlen(spin_str);
    ui_booster_off_dynamic__group_spin_off_text->color = spin_status_ ? 2 : 5;

    // 超级电容状态文字颜色
    ui_booster_off_dynamic__group_cap_charge_text->color = supercap_status_ ? 2 : 5;
}
