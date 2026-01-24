#include "dvc_referee.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "projdefs.h"
#include <cstdint>
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

}

void Referee::Task()
{
    ui_init_booster_off();
    for(;;)
    {
        // ui_booster_off_now_strings->color = 0;
        ui_update_booster_off();
        // 内部发送函数后已有10ms延时
        osDelay(pdMS_TO_TICKS(10));
    }

}
