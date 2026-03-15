#include "bmi088_middleware.h"
#include "stm32h723xx.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_spi_port.h"

namespace {
BspSpiHandle s_accel = nullptr;
BspSpiHandle s_gyro = nullptr;
BspSpiHandle s_active = nullptr;
}

void BMI088_GPIO_init(void)
{
    // GPIO 已由 Board 层完成初始化
}

void BMI088_com_init(void)
{
    if (s_accel && s_gyro) return;
    s_accel = bsp_spi_get(BSP_SPI_DEV_BMI088_ACCEL);
    s_gyro  = bsp_spi_get(BSP_SPI_DEV_BMI088_GYRO);
    configASSERT(s_accel != nullptr && s_gyro != nullptr);
    bsp_spi_set_mode(s_accel, BSP_SPI_BLOCK_MODE);
    bsp_spi_set_mode(s_gyro, BSP_SPI_BLOCK_MODE);
}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--) {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 480;
    told = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }
}

void BMI088_ACCEL_NS_L(void)
{
    s_active = s_accel;
    bsp_spi_select(s_active);
}

void BMI088_ACCEL_NS_H(void)
{
    if (s_active) {
        bsp_spi_deselect(s_active);
    }
    s_active = nullptr;
}

void BMI088_GYRO_NS_L(void)
{
    s_active = s_gyro;
    bsp_spi_select(s_active);
}

void BMI088_GYRO_NS_H(void)
{
    if (s_active) {
        bsp_spi_deselect(s_active);
    }
    s_active = nullptr;
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    configASSERT(s_active != nullptr);
    bsp_spi_transceive(s_active, &txdata, &rx_data, 1, 1);
    return rx_data;
}

