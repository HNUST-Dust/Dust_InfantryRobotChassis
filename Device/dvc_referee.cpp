#include "dvc_referee.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include <cstdint>
#include <cstring>
#include "crc.h"
#include "bsp_usart.h"
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
                // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&status_, tmp_buffer->data, sizeof(StatusData));
                
                has_received_rx_msg_ = true;

                // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            case kShootDataId:
            {
                // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&shoot_, tmp_buffer->data, sizeof(ShootData));
                // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            case kGameStatusId:
            {
                // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
                std::memcpy(&game_status_, tmp_buffer->data, sizeof(GameStatus));
                // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
                break;
            }
            default:
                break;
        }
        // 缓冲区直接推移
        i += 7 + tmp_buffer->data_length + 2;
    }
}

void print_message(const uint8_t *message, const int length) {

    // 等待 DMA 空闲
    while (!g_uart1_manage_object.tx_cplt_flag);
    memcpy(g_uart1_manage_object.tx_buffer, message, length);
    g_uart1_manage_object.tx_cplt_flag = false;
    HAL_UART_Transmit_DMA(
        g_uart1_manage_object.uart_handler, 
        g_uart1_manage_object.tx_buffer, 
        length
    );
}
#define DEFINE_FRAME_PROC(num, id)                          \
void Referee::ui_proc_ ## num##_frame(ui_ ## num##_frame_t *msg) {   \
    msg->header.SOF = 0xA5;                                 \
    msg->header.length = 6 + 15 * num;                      \
    msg->header.seq = seq++;                                \
    msg->header.crc8 = verify_crc8_check_sum((uint8_t*)msg, 4);        \
    msg->header.cmd_id = 0x0301;                            \
    msg->header.sub_id = id;                                \
    msg->header.send_id = ui_self_id;                       \
    msg->header.recv_id = ui_self_id + 256;                 \
    msg->crc16 = verify_crc16_check_sum((uint8_t*)msg, 13 + 15 * num); \
}

DEFINE_FRAME_PROC(1, 0x0101)
DEFINE_FRAME_PROC(2, 0x0102)
DEFINE_FRAME_PROC(5, 0x0103)
DEFINE_FRAME_PROC(7, 0x0104)

void Referee::ui_proc_string_frame(ui_string_frame_t *msg) {
    msg->header.SOF = 0xA5;
    msg->header.length = 51;
    msg->header.seq = seq++;
    msg->header.crc8 = verify_crc8_check_sum((uint8_t *) msg, 4);
    msg->header.cmd_id = 0x0301;
    msg->header.sub_id = 0x0110;
    msg->header.send_id = ui_self_id;
    msg->header.recv_id = ui_self_id + 256;
    msg->option.str_length = strlen(msg->option.string);
    msg->crc16 = verify_crc16_check_sum((uint8_t *) msg, 58);
}

void Referee::ui_init_0() 
{
    for (int i = 0; i < 2; i++) {
        ui_0_.data[i].figure_name[0] = 0;
        ui_0_.data[i].figure_name[1] = 0;
        ui_0_.data[i].figure_name[2] = i + 0;
        ui_0_.data[i].operate_type = 1;
    }
    // for (int i = 2; i < 2; i++) {
    //     ui_0_.data[i].operate_type = 0;
    // }

    ui_chassis_l_->figure_type = 0;
    ui_chassis_l_->operate_type = 1;
    ui_chassis_l_->layer = 0;
    ui_chassis_l_->color = 4;
    ui_chassis_l_->start_x = 0;
    ui_chassis_l_->start_y = 499;
    ui_chassis_l_->width = 3;
    ui_chassis_l_->end_x = 820;
    ui_chassis_l_->end_y = 646;

    ui_chassis_r_->figure_type = 0;
    ui_chassis_r_->operate_type = 1;
    ui_chassis_r_->layer = 0;
    ui_chassis_r_->color = 4;
    ui_chassis_r_->start_x = 1916;
    ui_chassis_r_->start_y = 500;
    ui_chassis_r_->width = 3;
    ui_chassis_r_->end_x = 1100;
    ui_chassis_r_->end_y = 645;

    ui_proc_2_frame(&ui_0_);
    SEND_MESSAGE((uint8_t *) &ui_0_, sizeof(ui_0_));
}

void Referee::ui_update_0() 
{
    for (int i = 0; i < 2; i++) {
        ui_0_.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_0_);
    SEND_MESSAGE((uint8_t *) &ui_0_, sizeof(ui_0_));
}

void Referee::ui_remove_0() 
{
    for (int i = 0; i < 2; i++) {
        ui_0_.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_0_);
    SEND_MESSAGE((uint8_t *) &ui_0_, sizeof(ui_0_));
}

void Referee::ui_init_1()
{
    ui_1_.option.figure_name[0] = 0;
    ui_1_.option.figure_name[1] = 0;
    ui_1_.option.figure_name[2] = 2;
    ui_1_.option.operate_type = 1;

    ui_spin_->figure_type = 7;
    ui_spin_->operate_type = 1;
    ui_spin_->layer = 0;
    ui_spin_->color = 5;
    ui_spin_->start_x = 110;
    ui_spin_->start_y = 760;
    ui_spin_->width = 2;
    ui_spin_->font_size = 20;
    ui_spin_->str_length = 4;
    strcpy(ui_spin_->string, "SPIN");

    ui_proc_string_frame(&ui_1_);
    SEND_MESSAGE((uint8_t *) &ui_1_, sizeof(ui_1_));

}

void Referee::ui_update_1()
{
    ui_1_.option.operate_type = 2;

    ui_spin_->color = spin_status_ ? 2 : 5;

    ui_proc_string_frame(&ui_1_);
    SEND_MESSAGE((uint8_t *) &ui_1_, sizeof(ui_1_));

}

void Referee::ui_remove_1()
{
    ui_1_.option.operate_type = 3;

    ui_proc_string_frame(&ui_1_);
    SEND_MESSAGE((uint8_t *) &ui_1_, sizeof(ui_1_));

}

void Referee::ui_init_2()
{
    ui_2_.option.figure_name[0] = 0;
    ui_2_.option.figure_name[1] = 0;
    ui_2_.option.figure_name[2] = 3;
    ui_2_.option.operate_type = 1;

    ui_booster_->figure_type = 7;
    ui_booster_->operate_type = 1;
    ui_booster_->layer = 0;
    ui_booster_->color = 5;
    ui_booster_->start_x = 110;
    ui_booster_->start_y = 720;
    ui_booster_->width = 2;
    ui_booster_->font_size = 20;
    ui_booster_->str_length = 7;
    strcpy(ui_booster_->string, "BOOSTER");

    ui_proc_string_frame(&ui_2_);
    SEND_MESSAGE((uint8_t *) &ui_2_, sizeof(ui_2_));
}

void Referee::ui_update_2()
{
    ui_2_.option.operate_type = 2;

    ui_booster_->color = booster_status_ ? 2 : 5;

    ui_proc_string_frame(&ui_2_);
    SEND_MESSAGE((uint8_t *) &ui_2_, sizeof(ui_2_));

}

void Referee::ui_remove_2()
{
    ui_2_.option.operate_type = 3;

    ui_proc_string_frame(&ui_2_);
    SEND_MESSAGE((uint8_t *) &ui_2_, sizeof(ui_2_));

}

void Referee::ui_init_3()
{
    ui_3_.option.figure_name[0] = 0;
    ui_3_.option.figure_name[1] = 0;
    ui_3_.option.figure_name[2] = 6;
    ui_3_.option.operate_type = 1;

    ui_supercap_->figure_type = 7;
    ui_supercap_->operate_type = 1;
    ui_supercap_->layer = 0;
    ui_supercap_->color = 5;
    ui_supercap_->start_x = 110;
    ui_supercap_->start_y = 800;
    ui_supercap_->width = 2;
    ui_supercap_->font_size = 20;
    ui_supercap_->str_length = 8;
    strcpy(ui_supercap_->string, "SUPERCAP");

    ui_proc_string_frame(&ui_3_);
    SEND_MESSAGE((uint8_t *) &ui_3_, sizeof(ui_3_));
}

void Referee::ui_update_3()
{
    ui_3_.option.operate_type = 2;

    ui_supercap_->color = supercap_status_ ? 2 : 5;

    ui_proc_string_frame(&ui_3_);
    SEND_MESSAGE((uint8_t *) &ui_3_, sizeof(ui_3_));

}

void Referee::ui_remove_3()
{
    ui_3_.option.operate_type = 3;

    ui_proc_string_frame(&ui_3_);
    SEND_MESSAGE((uint8_t *) &ui_3_, sizeof(ui_3_));
}

void Referee::InitUI()
{
    ui_init_0();
    ui_init_1();
    ui_init_2();
    ui_init_3();
}

void Referee::UpdateUI()
{
    ui_update_0();
    ui_update_1();
    ui_update_2();
    ui_update_3();
}

void Referee::RemoveUI()
{
    ui_remove_0();
    ui_remove_1();
    ui_remove_2();
    ui_remove_3();
}

void Referee::Task()
{
    for (;;)
    {
        ui_self_id = status_.id;
        
        if (!ui_inited_ && has_received_rx_msg_)
        {
            InitUI();
            ui_inited_ = true;
        }

        if (ui_inited_)
        {
            UpdateUI();
        }
        osDelay(pdMS_TO_TICKS(100));
    }
}
