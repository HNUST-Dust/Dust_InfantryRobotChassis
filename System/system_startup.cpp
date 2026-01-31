#include "system_startup.h"

#include "AppWiring.h"
#include "AppContext.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"

#include "can_motor_tx_task.h"
#include "../Device/vofa_tx_task.h"
#include "../Device/ui_tx_task.h"
#include "../Device/motor_actuator_task.h"
#include "../Device/motor_actuator_config.hpp"

#include "../Communication/dvc_mcu_comm.h"
#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Interaction/ModulesContext.h"
#include "../Interaction/app_chassis.h"
#include "../Interaction/app_gimbal.h"

void Bsp_BringUp(void)
{
    // Start IO services ASAP (enable UART DMA idle, CAN RX IT, etc.)
    App_WirePlatformIo();
}

void Board_BringUp(void)
{
    // Board-level devices on PCB (LEDs, IMU power/reset, etc.)
}

void Modules_BringUp(void)
{
#ifdef __cplusplus
    // Motor CAN TxTask：订阅电机/DM 发送 Topic，统一发送到 CAN（发布即自动发送）
    static CanMotorTxTask s_can_motor_tx_task;
    (void)s_can_motor_tx_task.Start();

    static VofaTxTask s_vofa_tx_task;
    (void)s_vofa_tx_task.Start(BSP_UART7);

    static UiTxTask s_ui_tx_task;
    (void)s_ui_tx_task.Start(BSP_UART1);

    // Ensure singletons are instantiated in one TU.
    Modules_Init();

    // 电机执行器层（方案B）：持有/初始化电机驱动，并订阅 actuator 命令输出到底层 Tx Topics
    {
        motor_cfg::Config cfg{};
        Modules_MotorActuator().BindConfig(cfg);
        Modules_MotorActuator().Bind(bsp_can_get(BSP_CAN_BUS1), bsp_can_get(BSP_CAN_BUS2), bsp_can_get(BSP_CAN_BUS3));
        (void)Modules_MotorActuator().Start();
    }

    // 启动底盘/云台（业务模块：只发布 actuator 命令 + 状态 topic）
    Modules_Chassis().Start();
    Modules_Gimbal().Start();

    // 上下板通讯组件初始化（CAN2）
    Modules_McuComm().Bind(bsp_can_get(BSP_CAN_BUS2), 0x01, 0x00);
    Modules_McuComm().Start();

    // 超级电容初始化（CAN3）
    Modules_Supercap().Bind(bsp_can_get(BSP_CAN_BUS3), 0x100, 0x003);
    Modules_Supercap().Start();

    // 裁判系统初始化（UART1 RX already started in Bsp_BringUp）
    Modules_Referee().Bind(bsp_uart_get(BSP_UART1));
    Modules_Referee().Start();
#endif
}

void App_Start(void)
{
    // App bring-up: start Robot (tasks), start control loops, etc.
    // Robot 不负责外设/模块 bring-up；仅做融合与发布，并通过 Start() 启动自己的任务。
    App_Init();
}

void System_Boot(void)
{
    Bsp_BringUp();
    Board_BringUp();
    Modules_BringUp();
    App_Start();
}
