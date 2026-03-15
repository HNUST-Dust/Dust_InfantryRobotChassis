/**
 * @file bsp_spi_port.h
 * @brief 上层可用的 SPI 端口抽象（HAL-free，C ABI）
 *
 * 设计目标：
 * =========
 * - 参考 bsp_uart_port：将 HAL/SPI/GPIO 细节收敛到 Board 层（Port 目录）。
 * - 上层模块（Device/App/Drivers）只依赖此抽象，避免直接 include main.h / spi.h / gpio.h。
 * - 零堆分配：句柄由 Board 层静态提供（或静态池提供）。
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct BspSpiOpaque* BspSpiHandle;

// 当前工程先覆盖已知外设（BMI088）。后续有新从机再在此扩展。
typedef enum {
    BSP_SPI_DEV_BMI088_ACCEL = 1,
    BSP_SPI_DEV_BMI088_GYRO  = 2,
} BspSpiDevId;

typedef enum {
    BSP_SPI_BLOCK_MODE = 0,
    BSP_SPI_IT_MODE    = 1,
    BSP_SPI_DMA_MODE   = 2,
} BspSpiTxRxMode;

typedef enum {
    BSP_SPI_EVENT_TX = 0,
    BSP_SPI_EVENT_RX = 1,
    BSP_SPI_EVENT_TXRX = 2,
    BSP_SPI_EVENT_ERROR = 3,
} BspSpiEvent;

typedef void (*BspSpiCallback)(BspSpiHandle h, BspSpiEvent event, uint16_t len, void* user_ctx);

// 获取一个从机句柄（由 Board 层固定映射）。
BspSpiHandle bsp_spi_get(BspSpiDevId id);

// 可选初始化：注册回调（IT/DMA 模式下完成/错误回调触发；BLOCK 模式通常不会触发）。
void bsp_spi_init(BspSpiHandle h, BspSpiCallback cb, void* user_ctx);

// 设定工作模式（可选；若 Board 不支持对应模式，可能会被忽略）。
void bsp_spi_set_mode(BspSpiHandle h, BspSpiTxRxMode mode);

// 片选控制：由 Board 层根据句柄的 CS 引脚实现。
void bsp_spi_select(BspSpiHandle h);
void bsp_spi_deselect(BspSpiHandle h);

// SPI 收发（不隐式控制片选）：
// - 允许上层在一次事务里保持 CS 低电平，完成多字节连续传输。
// - timeout_ms 仅对 BLOCK 模式生效（IT/DMA 由 Board 决定实现策略）。
bool bsp_spi_transmit(BspSpiHandle h, const uint8_t* tx, uint16_t len, uint32_t timeout_ms);
bool bsp_spi_receive(BspSpiHandle h, uint8_t* rx, uint16_t len, uint32_t timeout_ms);
bool bsp_spi_transceive(BspSpiHandle h, const uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout_ms);

// 异步接口：根据 mode(IT/DMA) 发起一次收发，返回 true 表示成功启动。
// - len 为本次事务字节数，用于回调透传。
// - 上层自行管理片选与 buffer 生命周期。
bool bsp_spi_transmit_async(BspSpiHandle h, const uint8_t* tx, uint16_t len);
bool bsp_spi_receive_async(BspSpiHandle h, uint8_t* rx, uint16_t len);
bool bsp_spi_transceive_async(BspSpiHandle h, const uint8_t* tx, uint8_t* rx, uint16_t len);

#ifdef __cplusplus
}
#endif
