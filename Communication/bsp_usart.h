/**
 * @file bsp_usart.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_BSP_UART_H_
#define BSP_BSP_UART_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "usart.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define UART_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UartCallback)(uint8_t *buffer, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
struct UartManageObject
{
    UART_HandleTypeDef *uart_handler;
    uint8_t tx_buffer[UART_BUFFER_SIZE];
    uint8_t rx_buffer[UART_BUFFER_SIZE];
    uint16_t rx_buffer_length;
    UartCallback callback_function;
    bool tx_cplt_flag;
};

/* Exported variables --------------------------------------------------------*/

// extern bool init_finished;

extern UartManageObject g_uart1_manage_object;
extern UartManageObject g_uart2_manage_object;
extern UartManageObject g_uart3_manage_object;
extern UartManageObject g_uart4_manage_object;
extern UartManageObject g_uart5_manage_object;
extern UartManageObject g_uart6_manage_object;
extern UartManageObject g_uart7_manage_object;
extern UartManageObject g_uart8_manage_object;

/* Exported function declarations --------------------------------------------*/

void uart_init(UART_HandleTypeDef *huart, UartCallback callback_function, uint16_t rx_buffer_length);

void uart_reinit(UART_HandleTypeDef *huart);

uint8_t uart_send_data(UART_HandleTypeDef *huart, uint8_t *daata, uint16_t length);

#endif

/************************ COPYRIGHT(C) HNUST-DUST **************************/
