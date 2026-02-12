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
#include "../daemon_supervisor/supervisor.hpp"
#include "../Drivers/can_tx_task.h"
#include "../Drivers/uart_tx_task.h"
#include "../communication_topic/actuator_cmd_topics.hpp"
#include "../Communication/dvc_MCU_comm.h"
#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Device/debug_tools.h"
#include "../Device/motor_ids.hpp"
#include "motors/dji_c6xx.hpp"
#include "motors/dm_mit.hpp"
#include "../App/app_chassis.h"
#include "../App/app_gimbal.h"

namespace {
static void daemon_system_fault(DaemonClient&)
{
    // FATAL: try to put system into a safe state (best-effort)
    Chassis::ExitGlobal();
    Gimbal::ExitGlobal();

    // Also request actuator-layer exit for DM motors (best-effort)
    // Motor module is business-decoupled; address by bus + motor id.
    orb::DmMitAdminCmd c{};
    c.bus = orb::CanBus::CAN3;
    c.op = orb::DmMitAdminOp::Exit;

    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
    orb::dm_mit_admin_cmd.publish(c);

    c.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
    orb::dm_mit_admin_cmd.publish(c);
}
} // namespace

void Bsp_BringUp(void)
{
    // DWT timeline is used by multiple modules (daemon_task, control loops).
    // Keep this early so any task can safely call dwt_get_timeline_ms().
    dwt_init(480);
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
    // Daemon supervisor must be initialized before any module registers clients.
    DaemonSupervisor::set_system_fault_hook(daemon_system_fault);
    DaemonSupervisor::Start();

    // Unified CAN/UART Tx tasks: the only modules allowed to call bsp_can_send/bsp_uart_send.
    static CanTxTask s_can_tx_task;
    s_can_tx_task.Start();

    static UartTxTask s_uart_tx_task;
    s_uart_tx_task.Start();

    // 直接初始化电机实例（逐个 Init）；业务模块仅通过 Topic 通讯。
    {
        auto* can1 = bsp_can_get(BSP_CAN_BUS1);
        configASSERT(can1 != nullptr);

        actuator::drivers::DjiC6xxMin::Config c{};
        c.bus = orb::CanBus::CAN1;
        c.method = actuator::drivers::DjiC6xxMin::ControlMethod::Omega;

        // TODO: 如需调参，后续可以从配置文件/编译宏读取
        c.gearbox_ratio = 1.0f;
        c.kp = 0.0f;
        c.ki = 0.0f;
        c.kd = 0.0f;
        c.current_limit = 20.0f;

        auto c1 = c;
        c1.rx_std_id = static_cast<uint16_t>(motor_ids::kWheel1);
        auto c2 = c;
        c2.rx_std_id = static_cast<uint16_t>(motor_ids::kWheel2);
        auto c3 = c;
        c3.rx_std_id = static_cast<uint16_t>(motor_ids::kWheel3);
        auto c4 = c;
        c4.rx_std_id = static_cast<uint16_t>(motor_ids::kWheel4);


        actuator::instances::dji_201.Init(can1, c1);
        actuator::instances::dji_201.SetTargetOmega(0.0f);
        actuator::instances::dji_201.JoinRuntime();

        actuator::instances::dji_202.Init(can1, c2);
        actuator::instances::dji_202.SetTargetOmega(0.0f);
        actuator::instances::dji_202.JoinRuntime();

        actuator::instances::dji_203.Init(can1, c3);
        actuator::instances::dji_203.SetTargetOmega(0.0f);
        actuator::instances::dji_203.JoinRuntime();

        actuator::instances::dji_204.Init(can1, c4);
        actuator::instances::dji_204.SetTargetOmega(0.0f);
        actuator::instances::dji_204.JoinRuntime();
    }

    {
        auto* can3 = bsp_can_get(BSP_CAN_BUS3);
        configASSERT(can3 != nullptr);

        actuator::drivers::DmMitMin::Config yaw_cfg{};
        yaw_cfg.bus = orb::CanBus::CAN3;
        yaw_cfg.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalYaw & 0x0F);
        yaw_cfg.master_id = 0x01;
        yaw_cfg.angle_max = 12.56637f;

        actuator::drivers::DmMitMin::Config pit_cfg{};
        pit_cfg.bus = orb::CanBus::CAN3;
        pit_cfg.can_rx_id = static_cast<uint8_t>(motor_ids::kGimbalPitch & 0x0F);
        pit_cfg.master_id = 0x02;

        // Motor-level init + bring-up + join runtime (blocking, best-effort).
        // NOTE: BringUpDefault() 内部使用 DWT delay，因此可在 osKernelStart() 前运行。
        {
            auto& yaw = actuator::instances::dm_01;
            yaw.Init(can3, yaw_cfg);
            yaw.BringUpDefault();
            yaw.JoinRuntime();
        }
        {
            auto& pit = actuator::instances::dm_02;
            pit.Init(can3, pit_cfg);
            pit.BringUpDefault();
            pit.JoinRuntime();
        }
    }

    // 上下板通讯组件初始化（CAN2）
    McuComm::Instance().Bind(orb::CanBus::CAN2, bsp_can_get(BSP_CAN_BUS2), 0x01, 0x00);
    McuComm::Instance().Start();

    // 超级电容初始化（CAN3）
    Supercap::Instance().Bind(orb::CanBus::CAN3, bsp_can_get(BSP_CAN_BUS3), 0x100, 0x003);
    Supercap::Instance().Start();

    // 裁判系统初始化（UART1 RX already started in Bsp_BringUp）
    Referee::Instance().Bind(bsp_uart_get(BSP_UART1), orb::UartPort::U1);
    Referee::Instance().Start();

    // VOFA 初始化（UART7 RX already started in Bsp_BringUp）
    DebugTools::Instance().Bind(bsp_uart_get(BSP_UART7), orb::UartPort::U7);
    DebugTools::Instance().Start();
#endif
}

void App_Start(void)
{
    // Distributed architecture: module tasks already started in Modules_BringUp().
    // 启动底盘/云台（业务模块：只发布 actuator 命令 + 状态 topic）
    Chassis::StartGlobal();
    Gimbal::StartGlobal();
}

void startup_thread(void *argument)
{
    Bsp_BringUp();
    Board_BringUp();
    Modules_BringUp();
    App_Start();
    vTaskDelete(NULL);
}