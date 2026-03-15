#include "bsp_spi_port.h"

// Board / HAL
#include "main.h"
#include "spi.h"

struct BspSpiOpaque {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    BspSpiTxRxMode mode;
    BspSpiCallback cb;
    void* user_ctx;
    uint16_t last_len;
};

static BspSpiOpaque s_bmi088_accel{&hspi2, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, BSP_SPI_BLOCK_MODE, nullptr, nullptr, 0};
static BspSpiOpaque s_bmi088_gyro{&hspi2, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, BSP_SPI_BLOCK_MODE, nullptr, nullptr, 0};

// SPI 回调路由：同一条 SPI 总线上同一时刻只能有一个事务在飞。
// 因此用“最近一次启动事务的从机句柄”作为 active 进行回调分发。
static BspSpiOpaque* s_active_hspi2 = nullptr;

static inline void set_active(BspSpiOpaque* impl)
{
    if (!impl || !impl->hspi) return;
    if (impl->hspi == &hspi2) s_active_hspi2 = impl;
}

static inline BspSpiOpaque* from_hal(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi2) return s_active_hspi2;
    return nullptr;
}

static inline BspSpiOpaque* to_impl(BspSpiHandle h)
{
    return reinterpret_cast<BspSpiOpaque*>(h);
}

BspSpiHandle bsp_spi_get(BspSpiDevId id)
{
    switch (id) {
        case BSP_SPI_DEV_BMI088_ACCEL: return reinterpret_cast<BspSpiHandle>(&s_bmi088_accel);
        case BSP_SPI_DEV_BMI088_GYRO:  return reinterpret_cast<BspSpiHandle>(&s_bmi088_gyro);
        default: return nullptr;
    }
}

void bsp_spi_init(BspSpiHandle h, BspSpiCallback cb, void* user_ctx)
{
    auto* impl = to_impl(h);
    if (!impl) return;
    impl->cb = cb;
    impl->user_ctx = user_ctx;
}

void bsp_spi_set_mode(BspSpiHandle h, BspSpiTxRxMode mode)
{
    auto* impl = to_impl(h);
    if (!impl) return;
    impl->mode = mode;
}

void bsp_spi_select(BspSpiHandle h)
{
    auto* impl = to_impl(h);
    if (!impl || !impl->cs_port) return;
    HAL_GPIO_WritePin(impl->cs_port, impl->cs_pin, GPIO_PIN_RESET);
}

void bsp_spi_deselect(BspSpiHandle h)
{
    auto* impl = to_impl(h);
    if (!impl || !impl->cs_port) return;
    HAL_GPIO_WritePin(impl->cs_port, impl->cs_pin, GPIO_PIN_SET);
}

static bool tx_impl(BspSpiOpaque* impl, const uint8_t* tx, uint16_t len, uint32_t timeout_ms)
{
    if (!impl || !impl->hspi || !tx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_BLOCK_MODE) {
        return (HAL_SPI_Transmit(impl->hspi, const_cast<uint8_t*>(tx), len, timeout_ms) == HAL_OK);
    }
    return false;
}

static bool rx_impl(BspSpiOpaque* impl, uint8_t* rx, uint16_t len, uint32_t timeout_ms)
{
    if (!impl || !impl->hspi || !rx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_BLOCK_MODE) {
        return (HAL_SPI_Receive(impl->hspi, rx, len, timeout_ms) == HAL_OK);
    }
    return false;
}

static bool txrx_impl(BspSpiOpaque* impl, const uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout_ms)
{
    if (!impl || !impl->hspi || !tx || !rx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_BLOCK_MODE) {
        return (HAL_SPI_TransmitReceive(impl->hspi, const_cast<uint8_t*>(tx), rx, len, timeout_ms) == HAL_OK);
    }
    return false;
}

bool bsp_spi_transmit(BspSpiHandle h, const uint8_t* tx, uint16_t len, uint32_t timeout_ms)
{
    return tx_impl(to_impl(h), tx, len, timeout_ms);
}

bool bsp_spi_receive(BspSpiHandle h, uint8_t* rx, uint16_t len, uint32_t timeout_ms)
{
    return rx_impl(to_impl(h), rx, len, timeout_ms);
}

bool bsp_spi_transceive(BspSpiHandle h, const uint8_t* tx, uint8_t* rx, uint16_t len, uint32_t timeout_ms)
{
    return txrx_impl(to_impl(h), tx, rx, len, timeout_ms);
}

static bool tx_async_impl(BspSpiOpaque* impl, const uint8_t* tx, uint16_t len)
{
    if (!impl || !impl->hspi || !tx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_IT_MODE) {
        return (HAL_SPI_Transmit_IT(impl->hspi, const_cast<uint8_t*>(tx), len) == HAL_OK);
    }
    if (impl->mode == BSP_SPI_DMA_MODE) {
        return (HAL_SPI_Transmit_DMA(impl->hspi, const_cast<uint8_t*>(tx), len) == HAL_OK);
    }
    return false;
}

static bool rx_async_impl(BspSpiOpaque* impl, uint8_t* rx, uint16_t len)
{
    if (!impl || !impl->hspi || !rx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_IT_MODE) {
        return (HAL_SPI_Receive_IT(impl->hspi, rx, len) == HAL_OK);
    }
    if (impl->mode == BSP_SPI_DMA_MODE) {
        return (HAL_SPI_Receive_DMA(impl->hspi, rx, len) == HAL_OK);
    }
    return false;
}

static bool txrx_async_impl(BspSpiOpaque* impl, const uint8_t* tx, uint8_t* rx, uint16_t len)
{
    if (!impl || !impl->hspi || !tx || !rx || len == 0) return false;
    impl->last_len = len;
    set_active(impl);

    if (impl->mode == BSP_SPI_IT_MODE) {
        return (HAL_SPI_TransmitReceive_IT(impl->hspi, const_cast<uint8_t*>(tx), rx, len) == HAL_OK);
    }
    if (impl->mode == BSP_SPI_DMA_MODE) {
        return (HAL_SPI_TransmitReceive_DMA(impl->hspi, const_cast<uint8_t*>(tx), rx, len) == HAL_OK);
    }
    return false;
}

bool bsp_spi_transmit_async(BspSpiHandle h, const uint8_t* tx, uint16_t len)
{
    return tx_async_impl(to_impl(h), tx, len);
}

bool bsp_spi_receive_async(BspSpiHandle h, uint8_t* rx, uint16_t len)
{
    return rx_async_impl(to_impl(h), rx, len);
}

bool bsp_spi_transceive_async(BspSpiHandle h, const uint8_t* tx, uint8_t* rx, uint16_t len)
{
    return txrx_async_impl(to_impl(h), tx, rx, len);
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    auto* impl = from_hal(hspi);
    if (!impl || !impl->cb) return;
    impl->cb(reinterpret_cast<BspSpiHandle>(impl), BSP_SPI_EVENT_TX, impl->last_len, impl->user_ctx);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    auto* impl = from_hal(hspi);
    if (!impl || !impl->cb) return;
    impl->cb(reinterpret_cast<BspSpiHandle>(impl), BSP_SPI_EVENT_RX, impl->last_len, impl->user_ctx);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    auto* impl = from_hal(hspi);
    if (!impl || !impl->cb) return;
    impl->cb(reinterpret_cast<BspSpiHandle>(impl), BSP_SPI_EVENT_TXRX, impl->last_len, impl->user_ctx);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    auto* impl = from_hal(hspi);
    if (!impl || !impl->cb) return;
    impl->cb(reinterpret_cast<BspSpiHandle>(impl), BSP_SPI_EVENT_ERROR, 0, impl->user_ctx);
}
