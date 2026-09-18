#ifndef FDCAN_RX_DISPATCHER_H_
#define FDCAN_RX_DISPATCHER_H_

#include "FDCAN_Basic.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FDCAN_RX_DISPATCHER_MAX_HANDLERS 8U

typedef uint8_t (*FDCAN_RxFrameHandler)(const FDCANFrame* frame, void* context);

typedef enum {
    FDCAN_RX_DISPATCH_OK = 0,
    FDCAN_RX_DISPATCH_INVALID_ARGUMENT,
    FDCAN_RX_DISPATCH_FULL,
    FDCAN_RX_DISPATCH_DUPLICATE,
    FDCAN_RX_DISPATCH_NOT_FOUND
} FDCAN_RxDispatchStatus;

typedef struct {
    volatile uint32_t callback_count;
    volatile uint32_t rx_count;
    volatile uint32_t handled_count;
    volatile uint32_t ignored_count;
    volatile uint32_t read_error_count;
    volatile uint32_t fifo_lost_count;
    volatile uint32_t fifo_full_count;
    volatile uint32_t last_interrupt_flags;
} FDCAN_RxDispatcherDiagnostics;

extern FDCAN_RxDispatcherDiagnostics fdcan_rx_dispatcher_diagnostics;

void FDCAN_RxDispatcher_Reset(void);
FDCAN_RxDispatchStatus FDCAN_RxDispatcher_Register(uint8_t can_bus,
                                                   FDCAN_RxFrameHandler handler,
                                                   void* context);
FDCAN_RxDispatchStatus FDCAN_RxDispatcher_Unregister(FDCAN_RxFrameHandler handler,
                                                     void* context);
uint8_t FDCAN_RxDispatcher_Dispatch(const FDCANFrame* frame);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_RX_DISPATCHER_H_ */
