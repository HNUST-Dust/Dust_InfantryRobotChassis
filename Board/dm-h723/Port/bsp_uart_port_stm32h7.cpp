#include "bsp_uart_port.h"

// Board / HAL
#include "usart.h"

struct BspUartOpaque {
    UART_HandleTypeDef* huart;
    BspUartRxCallback cb;
    uint8_t* rx_buf;
    uint16_t rx_len;
};

static uint8_t s_uart1_rx[512];
static uint8_t s_uart7_rx[512];

static BspUartOpaque s_uart1{&huart1, nullptr, s_uart1_rx, 0};
static BspUartOpaque s_uart7{&huart7, nullptr, s_uart7_rx, 0};

static inline BspUartOpaque* to_impl(BspUartHandle h) {
    return reinterpret_cast<BspUartOpaque*>(h);
}

static inline BspUartOpaque* from_hal(UART_HandleTypeDef* huart)
{
    if (huart == &huart1) return &s_uart1;
    if (huart == &huart7) return &s_uart7;
    return nullptr;
}

BspUartHandle bsp_uart_get(BspUartId id)
{
    switch (id)
    {
        case BSP_UART1: return reinterpret_cast<BspUartHandle>(&s_uart1);
        case BSP_UART7: return reinterpret_cast<BspUartHandle>(&s_uart7);
        default: return nullptr;
    }
}

static void start_rx(BspUartOpaque* impl)
{
    if (!impl || !impl->huart || !impl->rx_buf || impl->rx_len == 0) return;

    // Restart DMA RxToIdle
    (void)HAL_UARTEx_ReceiveToIdle_DMA(impl->huart, impl->rx_buf, impl->rx_len);

    // Avoid Half Transfer interrupts (optional but usually desired)
    if (impl->huart->hdmarx) {
        __HAL_DMA_DISABLE_IT(impl->huart->hdmarx, DMA_IT_HT);
    }
}

void bsp_uart_init(BspUartHandle h, BspUartRxCallback cb, uint16_t rx_buffer_length)
{
    auto* impl = to_impl(h);
    if (!impl) return;

    impl->cb = cb;

    // Clamp to internal buffer size
    constexpr uint16_t kBufSize = 512;
    impl->rx_len = (rx_buffer_length == 0) ? 0 : (rx_buffer_length > kBufSize ? kBufSize : rx_buffer_length);

    start_rx(impl);
}

bool bsp_uart_send(BspUartHandle h, uint8_t* data, uint16_t length)
{
    auto* impl = to_impl(h);
    if (!impl || !impl->huart) return false;
    return (HAL_UART_Transmit_DMA(impl->huart, data, length) == HAL_OK);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    auto* impl = from_hal(huart);
    if (!impl) return;

    if (impl->cb) {
        // Size is number of received bytes at IDLE event
        impl->cb(impl->rx_buf, Size);
    }

    start_rx(impl);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    auto* impl = from_hal(huart);
    if (!impl) return;

    start_rx(impl);
}
