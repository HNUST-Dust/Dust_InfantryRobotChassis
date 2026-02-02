/**
 * @file system_startup.cpp
 * @brief 工程统一启动序列实现（System_Boot + staged bring-up）
 *
 * 核心逻辑：
 * =========
 * - `System_Boot()` 按固定顺序执行：BSP -> Board -> Modules -> App。
 * - 将“模块拉起顺序”集中管理，避免分散在各处导致依赖关系难以审查。
 *
 * 关键依赖：
 * =========
 * - DWT 时间基在 Board_BringUp() 早期初始化，供 daemon/控制环路使用。
 * - daemon_supervisor 必须在各模块注册 client 前 init。
 * - CAN/UART 发送统一经由 TxTask：这里负责启动 `CanTxTask`/`UartTxTask` 作为唯一发送出口。
 *
 * 故障策略：
 * =========
 * - 通过 `DaemonSupervisor::set_system_fault_hook()` 设置系统级故障回调。
 *   当前实现为 best-effort：请求底盘/云台退出，并发布执行器层的管理指令。
 */

#include "system_startup.h"

#include "AppWiring.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"

#include "bsp_dwt.h"

// Daemon supervisor (watchdog-like health monitor)
#include "../daemon_supervisor/supervisor.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "../Drivers/can_tx_task.h"
#include "../Drivers/uart_tx_task.h"
#include "../Device/motor_actuator_task.h"
#include "../Device/motor_actuator_config.hpp"

#include "../Communication/dvc_MCU_comm.h"
#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Device/debug_tools.h"
#include "../Interaction/app_chassis.h"
#include "../Interaction/app_gimbal.h"

namespace {
static void daemon_system_fault(DaemonClient&)
{
    // FATAL: try to put system into a safe state (best-effort)
    Chassis_Instance().Exit();
    Gimbal_Instance().Exit();

    // Also request actuator-layer exit for DM motors (if MotorActuatorTask is alive)
    orb::GimbalDmAdminCmd admin{};
    admin.op = orb::GimbalDmAdminOp::BothExit;
    orb::gimbal_dm_admin_cmd.publish(admin);
}
} // namespace

void Bsp_BringUp(void)
{
    // Start IO services ASAP (enable UART DMA idle, CAN RX IT, etc.)
    App_WirePlatformIo();
}

void Board_BringUp(void)
{
    // Board-level devices on PCB (LEDs, IMU power/reset, etc.)

    // DWT timeline is used by multiple modules (daemon_task, control loops).
    // Keep this early so any task can safely call dwt_get_timeline_ms().
    dwt_init(480);
}

void Modules_BringUp(void)
{
#ifdef __cplusplus
    // Daemon supervisor must be initialized before any module registers clients.
    DaemonSupervisor::init();
    DaemonSupervisor::set_system_fault_hook(daemon_system_fault);

    // Start the daemon task (100Hz)
    {
        static StaticTask_t s_daemon_tcb;
        static StackType_t s_daemon_stack[256]; // 256 * 4 bytes = 1KB

        static const osThreadAttr_t kDaemonAttr = {
            .name = "daemon",
            .cb_mem = &s_daemon_tcb,
            .cb_size = sizeof(s_daemon_tcb),
            .stack_mem = s_daemon_stack,
            .stack_size = sizeof(s_daemon_stack),
            .priority = (osPriority_t)osPriorityAboveNormal,
        };

        (void)osThreadNew(daemon_task, nullptr, &kDaemonAttr);
    }

    // Unified CAN/UART Tx tasks: the only modules allowed to call bsp_can_send/bsp_uart_send.
    static CanTxTask s_can_tx_task;
    (void)s_can_tx_task.Start();

    static UartTxTask s_uart_tx_task;
    (void)s_uart_tx_task.Start();

    // 电机执行器层（方案B）：持有/初始化电机驱动，并订阅 actuator 命令输出到底层 Tx Topics
    {
        motor_cfg::Config cfg{};
        MotorActuatorTask_Instance().BindConfig(cfg);
        MotorActuatorTask_Instance().Bind(bsp_can_get(BSP_CAN_BUS1), bsp_can_get(BSP_CAN_BUS2), bsp_can_get(BSP_CAN_BUS3));
        (void)MotorActuatorTask_Instance().Start();
    }

    // 启动底盘/云台（业务模块：只发布 actuator 命令 + 状态 topic）
    Chassis_Instance().Start();
    Gimbal_Instance().Start();

    // 上下板通讯组件初始化（CAN2）
    McuComm_Instance().Bind(orb::CanBus::CAN2, bsp_can_get(BSP_CAN_BUS2), 0x01, 0x00);
    McuComm_Instance().Start();

    // 超级电容初始化（CAN3）
    Supercap_Instance().Bind(orb::CanBus::CAN3, bsp_can_get(BSP_CAN_BUS3), 0x100, 0x003);
    Supercap_Instance().Start();

    // 裁判系统初始化（UART1 RX already started in Bsp_BringUp）
    Referee_Instance().Bind(bsp_uart_get(BSP_UART1), orb::UartPort::U1);
    Referee_Instance().Start();

    // VOFA 初始化（UART7 RX already started in Bsp_BringUp）
    DebugTools_Instance().Bind(bsp_uart_get(BSP_UART7), orb::UartPort::U7);
    (void)DebugTools_Instance().Start();
#endif
}

void App_Start(void)
{
    // Distributed architecture: module tasks already started in Modules_BringUp().
}

void System_Boot(void)
{
    Bsp_BringUp();
    Board_BringUp();
    Modules_BringUp();
    App_Start();
}
