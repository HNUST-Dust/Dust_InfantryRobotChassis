/**
 * @file bsp_can.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_CAN_H_
#define BSP_CAN_H_
#ifdef __cplusplus
extern "C"{
#endif
/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "fdcan.h"
/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct CanRxBuffer
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
};

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CanCallback)(CanRxBuffer *);

/**
 * @brief CAN通信处理结构体
 *
 */
struct CanManageObject
{
    FDCAN_HandleTypeDef *can_handler;
    CanRxBuffer rx_buffer;
    CanCallback callback_function;
};

/* Exported variables ---------------------------------------------------------*/

extern FDCAN_HandleTypeDef g_hfdcan1;
extern FDCAN_HandleTypeDef g_hfdcan2;
extern FDCAN_HandleTypeDef g_hfdcan3;

extern CanManageObject g_can1_manage_object;
extern CanManageObject g_can2_manage_object;
extern CanManageObject g_can3_manage_object;

extern uint8_t g_can1_0x1ff_tx_data[];
extern uint8_t g_can1_0x200_tx_data[];
extern uint8_t g_can1_0x2ff_tx_data[];
extern uint8_t g_can1_0x3fe_tx_data[];
extern uint8_t g_can1_0x4fe_tx_data[];

extern uint8_t g_can2_0x1ff_tx_data[];
extern uint8_t g_can2_0x200_tx_data[];
extern uint8_t g_can2_0x2ff_tx_data[];
extern uint8_t g_can2_0x3fe_tx_data[];
extern uint8_t g_can2_0x4fe_tx_data[];

extern uint8_t g_can_supercap_tx_Data[];

/* Exported function declarations ---------------------------------------------*/

void can_init(FDCAN_HandleTypeDef *hcan, CanCallback callback_function);

void can_filter_mask_config(FDCAN_HandleTypeDef *hcan, uint8_t object_para, uint32_t id, uint32_t mask_id);

uint8_t can_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);
uint8_t fdcan_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint16_t length);

void can_period_elapsed_callback();
#ifdef __cplusplus
}
#endif
#endif // BSP_CAN_H_

/************************ COPYRIGHT(C) HNUST-DUST **************************/
