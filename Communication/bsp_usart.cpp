/**
 * @file bsp_usart.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "bsp_usart.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

UartManageObject g_uart1_manage_object = {0};
UartManageObject g_uart2_manage_object = {0};
UartManageObject g_uart3_manage_object = {0};
UartManageObject g_uart4_manage_object = {0};
UartManageObject g_uart5_manage_object = {0};
UartManageObject g_uart6_manage_object = {0};
UartManageObject g_uart7_manage_object = {0};
UartManageObject g_uart8_manage_object = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param callback_function 处理回调函数
 * @param rx_buffer_length 接收缓冲区长度
 */
void uart_init(UART_HandleTypeDef *huart, UartCallback callback_function, uint16_t rx_buffer_length)
{
    if (huart->Instance == USART1)
    {
        g_uart1_manage_object.uart_handler = huart;
        g_uart1_manage_object.callback_function = callback_function;
        g_uart1_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart1_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer, g_uart1_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART2)
    {
        g_uart2_manage_object.uart_handler = huart;
        g_uart2_manage_object.callback_function = callback_function;
        g_uart2_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart2_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer, g_uart2_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        g_uart3_manage_object.uart_handler = huart;
        g_uart3_manage_object.callback_function = callback_function;
        g_uart3_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart3_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer, g_uart3_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART4)
    {
        g_uart4_manage_object.uart_handler = huart;
        g_uart4_manage_object.callback_function = callback_function;
        g_uart4_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart4_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer, g_uart4_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        g_uart5_manage_object.uart_handler = huart;
        g_uart5_manage_object.callback_function = callback_function;
        g_uart5_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart5_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer, g_uart5_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        g_uart6_manage_object.uart_handler = huart;
        g_uart6_manage_object.callback_function = callback_function;
        g_uart6_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart6_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer, g_uart6_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        g_uart7_manage_object.uart_handler = huart;
        g_uart7_manage_object.callback_function = callback_function;
        g_uart7_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart7_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer, g_uart7_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART8)
    {
        g_uart8_manage_object.uart_handler = huart;
        g_uart8_manage_object.callback_function = callback_function;
        g_uart8_manage_object.rx_buffer_length = rx_buffer_length;
        g_uart8_manage_object.tx_cplt_flag = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer, g_uart8_manage_object.rx_buffer_length);
    }
}

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void uart_reinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer, g_uart1_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer, g_uart2_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer, g_uart3_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer, g_uart4_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer, g_uart5_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer, g_uart6_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer, g_uart7_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer, g_uart8_manage_object.rx_buffer_length);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t uart_send_data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(huart, data, length));
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // // 判断程序初始化完成
    // if (init_finished == false)
    // {
    //     return;
    // }

    // 选择回调函数
    if (huart->Instance == USART1)
    {
        if(g_uart1_manage_object.callback_function != nullptr)
        {
            g_uart1_manage_object.callback_function(g_uart1_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer, g_uart1_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART2)
    {
        if(g_uart2_manage_object.callback_function != nullptr)
        {
            g_uart2_manage_object.callback_function(g_uart2_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer, g_uart2_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        if(g_uart3_manage_object.callback_function != nullptr)
        {
            g_uart3_manage_object.callback_function(g_uart3_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer, g_uart3_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART4)
    {
        if(g_uart4_manage_object.callback_function != nullptr)
        {
            g_uart4_manage_object.callback_function(g_uart4_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer, g_uart4_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        if(g_uart5_manage_object.callback_function != nullptr)
        {
            g_uart5_manage_object.callback_function(g_uart5_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer, g_uart5_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        if(g_uart6_manage_object.callback_function != nullptr)
        {
            g_uart6_manage_object.callback_function(g_uart6_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer, g_uart6_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        if(g_uart7_manage_object.callback_function != nullptr)
        {
            g_uart7_manage_object.callback_function(g_uart7_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer, g_uart7_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART8)
    {
        if(g_uart8_manage_object.callback_function != nullptr)
        {
            g_uart8_manage_object.callback_function(g_uart8_manage_object.rx_buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer, g_uart8_manage_object.rx_buffer_length);
    }
}

/**
 * @brief HAL库UART错误中断
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart1_manage_object.rx_buffer, g_uart1_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart2_manage_object.rx_buffer, g_uart2_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart3_manage_object.rx_buffer, g_uart3_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_manage_object.rx_buffer, g_uart4_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart5_manage_object.rx_buffer, g_uart5_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart6_manage_object.rx_buffer, g_uart6_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart7_manage_object.rx_buffer, g_uart7_manage_object.rx_buffer_length);
    }
    else if (huart->Instance == UART8)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart8_manage_object.rx_buffer, g_uart8_manage_object.rx_buffer_length);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        g_uart7_manage_object.tx_cplt_flag = true;
    }
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
