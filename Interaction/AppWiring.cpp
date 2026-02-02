/**
 * @file AppWiring.cpp
 * @brief 平台 IO 装配实现：UART/CAN RX → 对应模块回调
 *
 * 核心逻辑：
 * =========
 * - 通过 `bsp_uart_init()` 注册 UART RX 回调。
 * - 通过 `bsp_can_add_rx_callback()` 注册 CAN RX 回调（支持多个订阅者扇出）。
 * - 回调内部只做“ID 分发/转发”，将数据交给模块实例处理。
 *
 * 数据流（以当前实现为准）：
 * ========================
 * - UART7: Debug/VOFA → `DebugTools_Instance().VofaReceiveCallback()`
 * - UART1: Referee → `Referee_Instance().RxCpltCallback()`
 * - CAN1/2/3: MotorActuatorTask → `OnCanXRx()`
 * - CAN2(特定 ID): 外部 MCU 数据 → `McuComm_Instance().Can*RxCpltCallback()`
 * - CAN3(0x100): Supercap → `Supercap_Instance().CanRxCpltCallback()`
 *
 * 注意事项：
 * =========
 * - 该文件不做 CAN/UART 发送；发送必须走统一的 TxTask/Topic 出口。
 * - 回调可能在中断上下文执行：必须避免阻塞与复杂计算。
 */

#include "AppWiring.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"

#include "app_chassis.h" // for Chassis_Instance()
#include "app_gimbal.h"  // for Gimbal_Instance()
#include "../Communication/dvc_MCU_comm.h"  // AUTOAIM_INFO_ID / REMOTE_CONTROL_ID / IMU_INFO_ID

#include "../Device/debug_tools.h"

#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Device/motor_actuator_task.h"  // MotorActuatorTask complete type for OnCanXRy

// Standalone modules: accessed via *_Instance() accessors

// USART7 debug
static void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    DebugTools_Instance().VofaReceiveCallback(buffer, length);
}

// USART1 referee
static void uart1_referee_callback(uint8_t *buffer, uint16_t length)
{
    Referee_Instance().RxCpltCallback(buffer, length);
}

// CAN RX fan-out callbacks (per-module)
static void can1_motor_callback(const BspCanFrame *frame)
{
    MotorActuatorTask_Instance().OnCan1Rx(frame);
}

static void can2_motor_callback(const BspCanFrame *frame)
{
    MotorActuatorTask_Instance().OnCan2Rx(frame);
}

static void can3_motor_callback(const BspCanFrame *frame)
{
    MotorActuatorTask_Instance().OnCan3Rx(frame);
}

static void can2_mcu_callback(const BspCanFrame *frame)
{
    switch (frame->id)
    {
        case AUTOAIM_INFO_ID:
            McuComm_Instance().CanAutoAimInfoRxCpltCallback(frame);
            break;
        case REMOTE_CONTROL_ID:
            McuComm_Instance().CanRemoteControlRxCpltCallback(frame);
            break;
        case IMU_INFO_ID:
            McuComm_Instance().CanImuInfoRxCpltCallback(frame);
            break;
        default:
            break;
    }
}

static void can3_supercap_callback(const BspCanFrame *frame)
{
    // supercap uses 0x100 by default
    if (frame->id == 0x100) {
        Supercap_Instance().CanRxCpltCallback(frame);
    }
}

void App_WirePlatformIo(void)
{
    constexpr uint16_t kUartRxBufferSize = 512;

    // UART
    bsp_uart_init(bsp_uart_get(BSP_UART7), uart7_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_UART1), uart1_referee_callback, kUartRxBufferSize);

    // CAN: register multiple RX subscribers (platform will fan-out)
    {
        auto* can1 = bsp_can_get(BSP_CAN_BUS1);
        auto* can2 = bsp_can_get(BSP_CAN_BUS2);
        auto* can3 = bsp_can_get(BSP_CAN_BUS3);

        (void)bsp_can_add_rx_callback(can1, can1_motor_callback);

        (void)bsp_can_add_rx_callback(can2, can2_mcu_callback);
        (void)bsp_can_add_rx_callback(can2, can2_motor_callback);

        (void)bsp_can_add_rx_callback(can3, can3_supercap_callback);
        (void)bsp_can_add_rx_callback(can3, can3_motor_callback);
    }
}
