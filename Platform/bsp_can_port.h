#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// App-layer CAN abstraction (HAL-free)

typedef struct BspCanOpaque* BspCanHandle;

typedef enum {
    BSP_CAN_BUS1 = 1,
    BSP_CAN_BUS2 = 2,
    BSP_CAN_BUS3 = 3,
} BspCanBus;

typedef enum {
    BSP_CAN_ID_STD = 0,
    BSP_CAN_ID_EXT = 1,
} BspCanIdType;

typedef enum {
    BSP_CAN_FRAME_DATA = 0,
    BSP_CAN_FRAME_REMOTE = 1,
} BspCanFrameType;

typedef struct {
    uint32_t id;           // 11-bit (STD) or 29-bit (EXT)
    uint8_t  len;          // 0..64
    uint8_t  data[64];

    // Flags
    BspCanIdType id_type;
    BspCanFrameType frame_type;
    bool is_fd;            // CAN FD frame
    bool brs;              // bit rate switching (FD)
    bool from_fifo1;       // true if received from FIFO1, false for FIFO0
} BspCanFrame;

typedef void (*BspCanRxCallback)(const BspCanFrame* frame);

// Get a bus handle. The returned handle is owned by the platform.
BspCanHandle bsp_can_get(BspCanBus bus);

// Start RX and register receive callback (legacy single-callback API).
// Note: This overwrites any previously registered callbacks.
void bsp_can_init(BspCanHandle h, BspCanRxCallback cb);

// Start RX (if not started yet) and APPEND a receive callback.
// Multiple callbacks will be invoked in registration order.
// Returns false if the internal callback list is full or handle is invalid.
bool bsp_can_add_rx_callback(BspCanHandle h, BspCanRxCallback cb);

// Send a frame.
bool bsp_can_send(BspCanHandle h, const BspCanFrame* tx);

#ifdef __cplusplus
}
#endif
