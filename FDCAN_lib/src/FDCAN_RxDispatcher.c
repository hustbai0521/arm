#include "FDCAN_RxDispatcher.h"

#include <string.h>

typedef struct {
    uint8_t can_bus;
    FDCAN_RxFrameHandler handler;
    void* context;
} FDCAN_RxDispatchEntry;

static FDCAN_RxDispatchEntry rx_handlers[FDCAN_RX_DISPATCHER_MAX_HANDLERS];

FDCAN_RxDispatcherDiagnostics fdcan_rx_dispatcher_diagnostics;

void FDCAN_RxDispatcher_Reset(void) {
    memset(rx_handlers, 0, sizeof(rx_handlers));
    memset(&fdcan_rx_dispatcher_diagnostics, 0, sizeof(fdcan_rx_dispatcher_diagnostics));
}

FDCAN_RxDispatchStatus FDCAN_RxDispatcher_Register(uint8_t can_bus,
                                                   FDCAN_RxFrameHandler handler,
                                                   void* context) {
    uint32_t i;
    int32_t free_index = -1;

    if (can_bus == 0U || handler == NULL || context == NULL) {
        return FDCAN_RX_DISPATCH_INVALID_ARGUMENT;
    }

    for (i = 0U; i < FDCAN_RX_DISPATCHER_MAX_HANDLERS; ++i) {
        if (rx_handlers[i].handler == handler && rx_handlers[i].context == context) {
            return FDCAN_RX_DISPATCH_DUPLICATE;
        }
        if (free_index < 0 && rx_handlers[i].handler == NULL) {
            free_index = (int32_t)i;
        }
    }

    if (free_index < 0) {
        return FDCAN_RX_DISPATCH_FULL;
    }

    rx_handlers[free_index].can_bus = can_bus;
    rx_handlers[free_index].context = context;
    rx_handlers[free_index].handler = handler;
    return FDCAN_RX_DISPATCH_OK;
}

FDCAN_RxDispatchStatus FDCAN_RxDispatcher_Unregister(FDCAN_RxFrameHandler handler,
                                                     void* context) {
    uint32_t i;

    if (handler == NULL || context == NULL) {
        return FDCAN_RX_DISPATCH_INVALID_ARGUMENT;
    }

    for (i = 0U; i < FDCAN_RX_DISPATCHER_MAX_HANDLERS; ++i) {
        if (rx_handlers[i].handler == handler && rx_handlers[i].context == context) {
            memset(&rx_handlers[i], 0, sizeof(rx_handlers[i]));
            return FDCAN_RX_DISPATCH_OK;
        }
    }
    return FDCAN_RX_DISPATCH_NOT_FOUND;
}

uint8_t FDCAN_RxDispatcher_Dispatch(const FDCANFrame* frame) {
    uint32_t i;

    if (frame == NULL) {
        return 0U;
    }

    for (i = 0U; i < FDCAN_RX_DISPATCHER_MAX_HANDLERS; ++i) {
        FDCAN_RxFrameHandler handler = rx_handlers[i].handler;
        if (handler != NULL && rx_handlers[i].can_bus == frame->canbus_id &&
            handler(frame, rx_handlers[i].context) != 0U) {
            return 1U;
        }
    }
    return 0U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_Device* device;
    FDCAN_RxHeaderTypeDef rx_header;
    FDCANFrame frame;
    uint8_t rx_data[8];

    device = NULL;
#ifdef USE_CAN_1
    if (hfdcan == can1.hfdcan) {
        device = &can1;
    }
#endif
    if (device == NULL) {
        return;
    }

    fdcan_rx_dispatcher_diagnostics.callback_count++;
    fdcan_rx_dispatcher_diagnostics.last_interrupt_flags = RxFifo0ITs;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        fdcan_rx_dispatcher_diagnostics.fifo_lost_count++;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U) {
        fdcan_rx_dispatcher_diagnostics.fifo_full_count++;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        memset(&rx_header, 0, sizeof(rx_header));
        memset(&frame, 0, sizeof(frame));
        memset(rx_data, 0, sizeof(rx_data));

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            fdcan_rx_dispatcher_diagnostics.read_error_count++;
            break;
        }

        fdcan_rx_dispatcher_diagnostics.rx_count++;
        /*
         * 脉塔和达妙使用 11 位标准帧，灵足使用 29 位扩展帧。
         * 这里只排除远程帧和非法 ID 类型，具体协议由各处理器再判断，
         * 避免在通用中断入口将灵足的扩展帧提前丢弃。
         */
        if ((rx_header.IdType != FDCAN_STANDARD_ID &&
             rx_header.IdType != FDCAN_EXTENDED_ID) ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            fdcan_rx_dispatcher_diagnostics.ignored_count++;
            continue;
        }

        frame.canbus_id = device->bus_id;
        frame.IDE = (rx_header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        frame.isRemote = 0;
        frame.Id.all = rx_header.Identifier;
        frame.Length = FDCAN_DataLengthToLength(rx_header.DataLength);
        if (frame.Length > sizeof(frame.Data.uchars)) {
            frame.Length = sizeof(frame.Data.uchars);
        }
        memcpy(frame.Data.uchars, rx_data, frame.Length);

        if (FDCAN_RxDispatcher_Dispatch(&frame) != 0U) {
            fdcan_rx_dispatcher_diagnostics.handled_count++;
        } else {
            fdcan_rx_dispatcher_diagnostics.ignored_count++;
        }
    }
}
