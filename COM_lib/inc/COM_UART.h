#ifndef COM_UART_H_
#define COM_UART_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COM_UART_MAX_PORTS 8U
#define COM_UART_RX_BUFFER_SIZE 128U

/* 回调在中断中执行；data 仅在回调期间有效，context 由应用层提供。 */
typedef void (*COM_UART_DataHandler)(const uint8_t* data, uint16_t length, void* context);

typedef struct {
    UART_HandleTypeDef* huart;
    COM_UART_DataHandler handler;
    void* context;
    uint8_t rx_buffer[COM_UART_RX_BUFFER_SIZE];
    uint16_t rx_length;  ///< 单次 Receive-to-IDLE 可写入的最大长度。
    uint8_t rx_byte;  ///< 最近一个完整接收块的末字节，仅用于调试观察。
    volatile uint8_t discard_current_rx;
    volatile uint32_t rx_block_count;
    volatile uint32_t error_count;
    volatile uint32_t last_error;
    volatile uint32_t restart_failure_count;
    volatile uint32_t overrun_error_count;
    volatile uint32_t frame_error_count;
    volatile uint32_t noise_error_count;
    volatile uint32_t parity_error_count;
} COM_UART_Port;

/* port 必须为静态生命周期对象。初始化/重新配置前应停止接收，
 * 注册操作在任务上下文串行调用；初始化不自动启动接收。
 * 同一 UART 不允许绑定多个 port；注册表满或参数无效返回 HAL_ERROR。 */
HAL_StatusTypeDef COM_UART_Init(COM_UART_Port* port, UART_HandleTypeDef* huart,
                               COM_UART_DataHandler handler, void* context);
/* 设置 Receive-to-IDLE 缓冲区使用长度；仅允许在接收停止时调用。 */
HAL_StatusTypeDef COM_UART_SetReceiveLength(COM_UART_Port* port, uint16_t length);
HAL_StatusTypeDef COM_UART_StartReceive(COM_UART_Port* port);
/* 接收已在进行时返回 HAL_OK；异常停止时重新挂起接收。 */
HAL_StatusTypeDef COM_UART_EnsureReceive(COM_UART_Port* port);
/* 终止接收并清除 ORE/NE/FE/PE，保留诊断计数。 */
HAL_StatusTypeDef COM_UART_StopReceive(COM_UART_Port* port);

#ifdef __cplusplus
}
#endif

#endif /* COM_UART_H_ */
