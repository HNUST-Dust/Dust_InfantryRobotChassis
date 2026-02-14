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
 * - UART7: Debug/VOFA → `DebugTools::Instance().VofaReceiveCallback()`
 * - UART1: Referee → `Referee::Instance().RxCpltCallback()`
 * - CAN1/2/3: Motors → 按电机 ID 分发到各电机实例 `CanRxCpltCallback()`
 * - CAN2(特定 ID): 外部 MCU 数据 → `McuComm::Instance().Can*RxCpltCallback()`
 * - CAN3(0x100): Supercap → `Supercap::Instance().CanRxCpltCallback()`
 *
 * 注意事项：
 * =========
 * - 该文件不做 CAN/UART 发送；发送必须走统一的 TxTask/Topic 出口。
 * - 回调可能在中断上下文执行：必须避免阻塞与复杂计算。
 */

#include "AppWiring.h"

#include "bsp_can_port.h"
#include "bsp_uart_port.h"

#include "../Communication/mcu_comm.h"  // AUTOAIM_INFO_ID / REMOTE_CONTROL_ID / IMU_INFO_ID

#include "../Device/debug_tools.h"

#include "../Device/supercap.h"
#include "../Device/dvc_referee.h"
#include "../Device/motor_ids.hpp"

#include "motors/dji_c6xx.hpp"
#include "motors/dm_mit.hpp"


// USART7 VOFA debug
static void uart7_debug_callback(uint8_t *buffer, uint16_t length)
{
    DebugTools::Instance().VofaReceiveCallback(buffer, length);
}

// USART1 裁判系统
static void uart1_referee_callback(uint8_t *buffer, uint16_t length)
{
    Referee::Instance().RxCpltCallback(buffer, length);
}

// 底盘电机
static void can1_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    // CAN1: chassis DJI motors. Pure ID dispatch (no default fan-out).
    switch (frame->id) {
    case motor_ids::kWheel1:
        actuator::instances::dji_201.CanRxCpltCallback(frame);
        break;
    case motor_ids::kWheel2:
        actuator::instances::dji_202.CanRxCpltCallback(frame);
        break;
    case motor_ids::kWheel3:
        actuator::instances::dji_203.CanRxCpltCallback(frame);
        break;
    case motor_ids::kWheel4:
        actuator::instances::dji_204.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

// 上下板通讯
static void can2_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    // CAN2: external MCU frames only.
    switch (frame->id) {
    case AUTOAIM_INFO_ID:
        McuComm::Instance().CanAutoAimInfoRxCpltCallback(frame);
        break;
    case REMOTE_CONTROL_ID:
        McuComm::Instance().CanRemoteControlRxCpltCallback(frame);
        break;
    case IMU_INFO_ID:
        McuComm::Instance().CanImuInfoRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

// 超级电容
// 云台yaw电机
// 云台pitch电机
static void can3_rx_callback(const BspCanFrame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->id_type != BSP_CAN_ID_STD || frame->frame_type != BSP_CAN_FRAME_DATA || frame->len < 8u) {
        return;
    }

    switch (frame->id) {
    case motor_ids::kSupercap:
        Supercap::Instance().CanRxCpltCallback(frame);
        break;
    case motor_ids::kGimbalPitch:
        actuator::instances::dm_02.CanRxCpltCallback(frame);
        break;
    case motor_ids::kGimbalYaw:
        actuator::instances::dm_01.CanRxCpltCallback(frame);
        break;
    default:
        break;
    }
}

void App_WirePlatformIo(void)
{
    constexpr uint16_t kUartRxBufferSize = 512;

    // UART
    bsp_uart_init(bsp_uart_get(BSP_UART7), uart7_debug_callback, kUartRxBufferSize);
    bsp_uart_init(bsp_uart_get(BSP_UART1), uart1_referee_callback, kUartRxBufferSize);

    // CAN
    auto* can1 = bsp_can_get(BSP_CAN_BUS1);
    auto* can2 = bsp_can_get(BSP_CAN_BUS2);
    auto* can3 = bsp_can_get(BSP_CAN_BUS3);

    (void)bsp_can_add_rx_callback(can1, can1_rx_callback);
    (void)bsp_can_add_rx_callback(can2, can2_rx_callback);
    (void)bsp_can_add_rx_callback(can3, can3_rx_callback);
}
