#pragma  once
#include "stdlib.h"
#include "stdint.h"
#include "ui.h"
class Referee{
public:
    void Init();
    void Task();
    void FreshDynamicUI();
    void FreshStaticUI();
    void RxCpltCallback(uint8_t *buffer, uint16_t length);
private:
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};