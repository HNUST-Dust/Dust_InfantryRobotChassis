#include "dvc_referee.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include "ui_interface.h"
#include <cstdint>
#include <cstring>
#include "crc.h"
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

    RefereeUartData *tmp_buffer;

    for (int i = 0; i < length;)
    {
        tmp_buffer = (RefereeUartData *) &buffer[i];

        // 未通过头校验
        if (tmp_buffer->frame_header != 0xA5)
        {
            i++;
            continue;
        }
        // 未通过CRC8校验, 顺一位继续判断
        // if (verify_crc8_check_sum((uint8_t *) tmp_buffer, 4) != tmp_buffer->crc_8)
        // {
        //     i++;
        //     continue;
        // }
        // 未通过CRC16校验, 跨过当前包继续判断
        // if (verify_crc16_check_sum((uint8_t *) tmp_buffer, 7 + tmp_buffer->data_length) != *(uint16_t *) ((uint32_t) tmp_buffer + 7 + tmp_buffer->data_length))
        // {
        //     i += 9 + tmp_buffer->data_length;
        //     continue;
        // }
        // 通过校验但帧不够长
        if (i + 7 + tmp_buffer->data_length + 2 > length)
        {
            break;
        }
        switch(tmp_buffer->referee_command_id)
        {
            case kStatusDataId:
            {
                UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&status_, tmp_buffer->data, sizeof(StatusData));
                
                has_received_rx_msg_ = true;

                taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            case kShootDataId:
            {
                UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&shoot_, tmp_buffer->data, sizeof(ShootData));
                taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            case kGameStatusId:
            {
                UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&game_status_, tmp_buffer->data, sizeof(GameStatus));
                taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            default:
                break;
        }
        // 缓冲区直接推移
        i += 7 + tmp_buffer->data_length + 2;
    }
}

void Referee::Task()
{
    for (;;)
    {
        FreshDynamicUI();

        if (!ui_inited_ && has_received_rx_msg_)
        {
            ui_init_booster_off();
            ui_inited_ = true;
        }

        if (ui_inited_)
        {
            ui_update_booster_off();
        }
        osDelay(pdMS_TO_TICKS(500));
    }
}

void Referee::FreshDynamicUI()
{
    // 从裁判系统状态包同步本机器人 ID
    if (status_.id != 0) {
        ui_self_id = status_.id;
        // ui_self_id = 3; // 目前先写死为1，后续根据实际情况修改
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
