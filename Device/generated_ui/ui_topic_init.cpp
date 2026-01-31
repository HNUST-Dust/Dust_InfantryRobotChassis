#include "ui_interface.h"

#include "../ui_tx_task.h"

// 可选初始化：把 UiTxTask 拉起（默认 UART1）。
// 注意：如果系统启动流程里已统一 Init，则这里可不调用。
void ui_tx_topic_init() {
    static UiTxTask s_ui_tx_task;
    s_ui_tx_task.Init(BSP_UART1);
}
