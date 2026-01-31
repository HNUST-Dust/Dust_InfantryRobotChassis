#include "bsp_can_port.h"

// Board / HAL
#include "fdcan.h"

#include <string.h>

struct BspCanOpaque {
    FDCAN_HandleTypeDef* hfdcan;

    // Multiple subscribers (no heap)
    static constexpr uint8_t kMaxRxCallbacks = 6;
    BspCanRxCallback cbs[kMaxRxCallbacks];
    uint8_t cb_count;
    bool started;
};

static BspCanOpaque s_can1{&hfdcan1, {nullptr}, 0, false};
static BspCanOpaque s_can2{&hfdcan2, {nullptr}, 0, false};
static BspCanOpaque s_can3{&hfdcan3, {nullptr}, 0, false};

static inline BspCanOpaque* to_impl(BspCanHandle h) {
    return reinterpret_cast<BspCanOpaque*>(h);
}

static inline BspCanOpaque* from_hal(FDCAN_HandleTypeDef* hfdcan)
{
    if (hfdcan == &hfdcan1) return &s_can1;
    if (hfdcan == &hfdcan2) return &s_can2;
    if (hfdcan == &hfdcan3) return &s_can3;
    return nullptr;
}

BspCanHandle bsp_can_get(BspCanBus bus)
{
    switch (bus)
    {
        case BSP_CAN_BUS1: return reinterpret_cast<BspCanHandle>(&s_can1);
        case BSP_CAN_BUS2: return reinterpret_cast<BspCanHandle>(&s_can2);
        case BSP_CAN_BUS3: return reinterpret_cast<BspCanHandle>(&s_can3);
        default:           return nullptr;
    }
}

static inline uint8_t dlc_to_len(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default:
            // Fallback for any non-standard values
            return (dlc <= 64) ? static_cast<uint8_t>(dlc) : 8;
    }
}

static void dispatch_rx(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo)
{
    auto* impl = from_hal(hfdcan);
    if (!impl || impl->cb_count == 0) return;

    FDCAN_RxHeaderTypeDef hdr{};
    uint8_t data[64]{};

    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &hdr, data) != HAL_OK) return;

    BspCanFrame f{};
    f.id = hdr.Identifier;
    f.id_type = (hdr.IdType == FDCAN_STANDARD_ID) ? BSP_CAN_ID_STD : BSP_CAN_ID_EXT;
    f.frame_type = (hdr.RxFrameType == FDCAN_DATA_FRAME) ? BSP_CAN_FRAME_DATA : BSP_CAN_FRAME_REMOTE;
    f.is_fd = (hdr.FDFormat == FDCAN_FD_CAN);
    f.brs = (hdr.BitRateSwitch == FDCAN_BRS_ON);
    f.from_fifo1 = (fifo == FDCAN_RX_FIFO1);

    f.len = dlc_to_len(hdr.DataLength);
    if (f.len > 0) {
        memcpy(f.data, data, f.len);
    }

    // fan-out
    for (uint8_t i = 0; i < impl->cb_count; ++i) {
        auto cb = impl->cbs[i];
        if (cb) {
            cb(&f);
        }
    }
}

static void ensure_started(BspCanOpaque* impl)
{
    if (!impl || impl->started) return;

    // Keep behavior aligned with previous wrapper: start and enable RX interrupts.
    (void)HAL_FDCAN_Start(impl->hfdcan);
    (void)HAL_FDCAN_ActivateNotification(impl->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    (void)HAL_FDCAN_ActivateNotification(impl->hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    impl->started = true;
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;
    dispatch_rx(hfdcan, FDCAN_RX_FIFO0);
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) return;
    dispatch_rx(hfdcan, FDCAN_RX_FIFO1);
}

void bsp_can_init(BspCanHandle h, BspCanRxCallback cb)
{
    auto* impl = to_impl(h);
    if (!impl) return;

    // legacy behavior: overwrite all callbacks
    impl->cb_count = 0;
    if (cb) {
        impl->cbs[0] = cb;
        impl->cb_count = 1;
    }

    ensure_started(impl);
}

bool bsp_can_add_rx_callback(BspCanHandle h, BspCanRxCallback cb)
{
    auto* impl = to_impl(h);
    if (!impl || !cb) return false;

    // reject duplicates (cheap linear scan, list is tiny)
    for (uint8_t i = 0; i < impl->cb_count; ++i) {
        if (impl->cbs[i] == cb) {
            ensure_started(impl);
            return true;
        }
    }

    if (impl->cb_count >= BspCanOpaque::kMaxRxCallbacks) {
        return false;
    }

    impl->cbs[impl->cb_count++] = cb;
    ensure_started(impl);
    return true;
}

bool bsp_can_send(BspCanHandle h, const BspCanFrame* tx)
{
    auto* impl = to_impl(h);
    if (!impl || !tx) return false;

    FDCAN_TxHeaderTypeDef th{};
    th.Identifier = tx->id;
    th.IdType = (tx->id_type == BSP_CAN_ID_STD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    th.TxFrameType = (tx->frame_type == BSP_CAN_FRAME_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
    th.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    th.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    th.MessageMarker = 0;

    if (tx->is_fd) {
        th.FDFormat = FDCAN_FD_CAN;
        th.BitRateSwitch = tx->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    } else {
        th.FDFormat = FDCAN_CLASSIC_CAN;
        th.BitRateSwitch = FDCAN_BRS_OFF;
    }

    // DataLength must be DLC code for FD; legacy code sometimes passed raw length for <=8.
    if (tx->len <= 8) {
        th.DataLength = tx->len;
    } else {
        switch (tx->len) {
            case 12: th.DataLength = FDCAN_DLC_BYTES_12; break;
            case 16: th.DataLength = FDCAN_DLC_BYTES_16; break;
            case 20: th.DataLength = FDCAN_DLC_BYTES_20; break;
            case 24: th.DataLength = FDCAN_DLC_BYTES_24; break;
            case 32: th.DataLength = FDCAN_DLC_BYTES_32; break;
            case 48: th.DataLength = FDCAN_DLC_BYTES_48; break;
            case 64: th.DataLength = FDCAN_DLC_BYTES_64; break;
            default:
                // Unsupported length for FD
                return false;
        }
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(impl->hfdcan, &th, const_cast<uint8_t*>(tx->data)) == HAL_OK);
}
