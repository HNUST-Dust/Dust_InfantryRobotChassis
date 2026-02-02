/**
 * @file bsp_uart_port.h
 * @brief 上层可用的 UART 端口抽象（HAL-free，C ABI）
 *
 * 设计思路：
 * =========
 * - 以 `BspUartHandle` 隐藏底层 UART/HAL/DMA 细节，向上提供 init/RX 回调与发送接口。
 * - 统一由 Board/Drivers 完成硬件初始化与 IRQ/DMA 配置，上层只依赖此抽象接口。
 *
 * 线程模型与约束：
 * ===============
 * - RX 回调通常在中断相关上下文触发（取决于具体实现）：回调必须轻量、不可阻塞。
 * - 工程架构约束：上层业务模块不应直接调用 `bsp_uart_send()`，应发布到 `orb::uart_tx`，由
 *   `Drivers/UartTxTask` 统一发送（唯一出口）。
 */

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
