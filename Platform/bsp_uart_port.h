#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct BspUartOpaque* BspUartHandle;

typedef enum {
    BSP_UART1 = 1,
    BSP_UART2 = 2,
    BSP_UART3 = 3,
    BSP_UART4 = 4,
    BSP_UART5 = 5,
    BSP_UART6 = 6,
    BSP_UART7 = 7,
    BSP_UART8 = 8,
    BSP_USART10 = 10,
} BspUartId;

typedef void (*BspUartRxCallback)(uint8_t* buffer, uint16_t length);

BspUartHandle bsp_uart_get(BspUartId id);

void bsp_uart_init(BspUartHandle h, BspUartRxCallback cb, uint16_t rx_buffer_length);

bool bsp_uart_send(BspUartHandle h, uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif
