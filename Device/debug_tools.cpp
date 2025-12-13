#include "debug_tools.h"
#include <cstdint>
#include <cstring>
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

void DebugTools::VofaInit(){

}

void DebugTools::VofaSendFloat(float data)
{
    // 等待 DMA 空闲
    while (!g_uart7_manage_object.tx_cplt_flag);
    memcpy(g_uart7_manage_object.tx_buffer, &data, sizeof(float));
    // uart_send_data(&huart7, data_buf, 4);
    // taskENTER_CRITICAL();
    g_uart7_manage_object.tx_cplt_flag = false;
    HAL_UART_Transmit_DMA(
        g_uart7_manage_object.uart_handler, 
        g_uart7_manage_object.tx_buffer, 
        4*sizeof(uint8_t)
    );
    // taskEXIT_CRITICAL();
}

void DebugTools::VofaSendTail()
{
    static uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    // uart_send_data(&huart7, tail, 4);
    while (!g_uart7_manage_object.tx_cplt_flag);
    // taskENTER_CRITICAL();
    g_uart7_manage_object.tx_cplt_flag = false;
    HAL_UART_Transmit_DMA(g_uart7_manage_object.uart_handler, tail, 4*sizeof(uint8_t));
    // taskEXIT_CRITICAL();

}

void DebugTools::VofaReceiveCallback(uint8_t *buffer, uint16_t length)
{

}