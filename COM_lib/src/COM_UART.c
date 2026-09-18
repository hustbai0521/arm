#include "COM_UART.h"
#include <stddef.h>
#include <string.h>

static COM_UART_Port* ports[COM_UART_MAX_PORTS];

static COM_UART_Port* COM_UART_FindPort(UART_HandleTypeDef* huart) {
    uint32_t i;
    for (i = 0U; i < COM_UART_MAX_PORTS; ++i) {
        if (ports[i] != NULL && ports[i]->huart == huart) {
            return ports[i];
        }
    }
    return NULL;
}

HAL_StatusTypeDef COM_UART_Init(COM_UART_Port* port, UART_HandleTypeDef* huart,
                               COM_UART_DataHandler handler, void* context) {
    uint32_t i;
    uint32_t slot = COM_UART_MAX_PORTS;

    if (port == NULL || huart == NULL || handler == NULL) {
        return HAL_ERROR;
    }
    for (i = 0U; i < COM_UART_MAX_PORTS; ++i) {
        if (ports[i] == port) {
            slot = i;
        } else if (ports[i] != NULL && ports[i]->huart == huart) {
            return HAL_ERROR;
        }
    }
    if (slot == COM_UART_MAX_PORTS) {
        for (i = 0U; i < COM_UART_MAX_PORTS; ++i) {
            if (ports[i] == NULL) {
                slot = i;
                break;
            }
        }
    }
    if (slot == COM_UART_MAX_PORTS) {
        return HAL_ERROR;
    }

    port->huart = huart;
    port->handler = handler;
    port->context = context;
    memset(port->rx_buffer, 0, sizeof(port->rx_buffer));
    port->rx_length = 1U;
    port->rx_byte = 0U;
    port->discard_current_rx = 0U;
    port->rx_block_count = 0U;
    port->error_count = 0U;
    port->last_error = HAL_UART_ERROR_NONE;
    port->restart_failure_count = 0U;
    port->overrun_error_count = 0U;
    port->frame_error_count = 0U;
    port->noise_error_count = 0U;
    port->parity_error_count = 0U;
    ports[slot] = port;
    return HAL_OK;
}

HAL_StatusTypeDef COM_UART_SetReceiveLength(COM_UART_Port* port, uint16_t length) {
    if (port == NULL || port->huart == NULL || COM_UART_FindPort(port->huart) != port ||
        length == 0U || length > COM_UART_RX_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    if (port->huart->RxState == HAL_UART_STATE_BUSY_RX) {
        return HAL_BUSY;
    }
    port->rx_length = length;
    return HAL_OK;
}

HAL_StatusTypeDef COM_UART_StartReceive(COM_UART_Port* port) {
    HAL_StatusTypeDef status;
    if (port == NULL || port->huart == NULL || COM_UART_FindPort(port->huart) != port ||
        port->rx_length == 0U || port->rx_length > COM_UART_RX_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    status = HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->rx_buffer, port->rx_length);
    if (status != HAL_OK) {
        port->restart_failure_count++;
    }
    return status;
}

HAL_StatusTypeDef COM_UART_EnsureReceive(COM_UART_Port* port) {
    if (port == NULL || port->huart == NULL || COM_UART_FindPort(port->huart) != port) {
        return HAL_ERROR;
    }
    if (port->huart->RxState == HAL_UART_STATE_BUSY_RX) {
        return HAL_OK;
    }
    return COM_UART_StartReceive(port);
}

HAL_StatusTypeDef COM_UART_StopReceive(COM_UART_Port* port) {
    HAL_StatusTypeDef status;
    UART_HandleTypeDef* huart;
    if (port == NULL || port->huart == NULL || COM_UART_FindPort(port->huart) != port) {
        return HAL_ERROR;
    }
    huart = port->huart;
    status = HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    return status;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    COM_UART_Port* port = COM_UART_FindPort(huart);

    if (port != NULL) {
        /*
         * HAL 可能在带错误的同一次 IRQ 中先完成接收。如果此处立即重装，
         * UART_Start_Receive_IT 会把 ErrorCode 清零，导致错误回调丢失根因。
         */
        if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
            return;
        }

        if (size > port->rx_length) {
            size = port->rx_length;
        }
        if (port->discard_current_rx == 0U && size > 0U) {
            port->rx_byte = port->rx_buffer[size - 1U];
            port->rx_block_count++;
            port->handler(port->rx_buffer, size, port->context);
        }
        port->discard_current_rx = 0U;
        (void)COM_UART_StartReceive(port);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    COM_UART_Port* port = COM_UART_FindPort(huart);
    uint32_t error;

    if (port != NULL) {
        error = huart->ErrorCode;
        if (error == HAL_UART_ERROR_NONE) {
            return;
        }

        port->error_count++;
        port->last_error = error;
        if ((error & HAL_UART_ERROR_ORE) != 0U) {
            port->overrun_error_count++;
        }
        if ((error & HAL_UART_ERROR_FE) != 0U) {
            port->frame_error_count++;
        }
        if ((error & HAL_UART_ERROR_NE) != 0U) {
            port->noise_error_count++;
        }
        if ((error & HAL_UART_ERROR_PE) != 0U) {
            port->parity_error_count++;
        }

        /*
         * ORE/RTO/DMA 是阻塞错误，HAL 在调用本回调前已结束本次接收；
         * 直接重装下一块。FE/NE/PE 为非阻塞错误，保留当前接收过程但丢弃
         * 整块，避免把已损坏的数据交给协议解析器。
         */
        if (((error & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO | HAL_UART_ERROR_DMA)) != 0U) ||
            huart->RxState != HAL_UART_STATE_BUSY_RX) {
            port->discard_current_rx = 0U;
            /* AbortReceive 同时清错误标志并冲掉 FIFO 中属于坏帧的残留字节。 */
            if (COM_UART_StopReceive(port) == HAL_OK) {
                (void)COM_UART_StartReceive(port);
            } else {
                port->restart_failure_count++;
            }
        } else {
            port->discard_current_rx = 1U;
        }
    }
}
