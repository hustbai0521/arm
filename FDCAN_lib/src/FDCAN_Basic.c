#include "FDCAN_Basic.h"
#include "FDCAN_Proplist.h"
#include "fdcan.h"
#include "string.h"

// -> 命中过滤器
// -> 放进 FIFO0 或 FIFO1
// -> 触发 FIFO0 new message / FIFO1 new message 事件
// -> 这个事件再被路由到 Interrupt 0 或 Interrupt 1
// -> MCU 进入对应 IRQHandler
// -> 在回调里 HAL_FDCAN_GetRxMessage(...) 把数据从 FIFO 取出来

// 报文进 FIFO0
// -> 产生 RX FIFO0 new message 事件
// -> 这个事件被使能
// -> 这个事件被分配到 Interrupt 0 或 Interrupt 1
// -> 进入对应 IRQHandler
// -> HAL_FDCAN_IRQHandler(...)
// -> HAL_FDCAN_RxFifo0Callback(...)

#ifdef USE_CAN_1
FDCAN_Device can1 = {
    .hfdcan = &hfdcan1,
    .bus_id = 1,
};
#endif
// 当前只使用 FDCAN1，注释 FDCAN2 设备定义，避免引用未配置的 hfdcan2。
// #ifdef USE_CAN_2
// FDCAN_Device can2 = {&hfdcan2, 2, 0, 0};
// #endif

FDCAN_Device* FDCAN_GetDevice(uint8_t bus_id) {
    switch (bus_id) {
#ifdef USE_CAN_1
        case 1:
            return &can1;
#endif
        // 当前只使用 FDCAN1，注释 FDCAN2 选择分支；bus_id 为 2 时返回 NULL。
// #ifdef USE_CAN_2
//         case 2:
//             return &can2;
// #endif
        default:
            return NULL;
    }
}

static uint32_t FDCAN_GetHalFifo(FDCAN_RX_FIFO_Type fifo) {
    if (fifo == CAN_RX_FIFO1)
        return FDCAN_FILTER_TO_RXFIFO1;
    return FDCAN_FILTER_TO_RXFIFO0;
}

static uint32_t FDCAN_GetBehavior(FDCAN_RX_FIFO_Type fifo) {
    if (fifo == CAN_RX_FIFO1)
        return FDCAN_ACCEPT_IN_RX_FIFO1;
    return FDCAN_ACCEPT_IN_RX_FIFO0;
}

static uint32_t FDCAN_GetActiveITs(FDCAN_RX_FIFO_Type fifo) {
    uint32_t Act = FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING;
    if (fifo == CAN_RX_FIFO0 || fifo == CAN_RX_FIFO_BOTH)
        Act |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
    if (fifo == CAN_RX_FIFO1 || fifo == CAN_RX_FIFO_BOTH)
        Act |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL | FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    return Act;
}

static FDCAN_Device* FDCAN_GetDeviceByHandle(FDCAN_HandleTypeDef* hfdcan) {
#ifdef USE_CAN_1
    if (hfdcan == can1.hfdcan) {
        return &can1;
    }
#endif
    return NULL;
}

static FDCAN_ErrorCode FDCAN_SetError(FDCAN_Device* dev, FDCAN_ErrorCode error) {
    if (dev != NULL) {
        dev->last_error = error;
        if (error != FDCAN_ERR_NONE) {
            dev->error_count++;
        }
    }
    return error;
}

void FDCAN_AddStdFilter(FDCAN_Device* dev, FDCAN_FilterTypeDef* sFilterConfig) {
    if (dev->std_filter_idx >= dev->hfdcan->Init.StdFiltersNbr) {
        return;
    }
    if (HAL_FDCAN_ConfigFilter(dev->hfdcan, sFilterConfig) != HAL_OK) {
        return;
    }
    dev->std_filter_idx++;
}

void FDCAN_AddExtFilter(FDCAN_Device* dev, FDCAN_FilterTypeDef* sFilterConfig) {
    if (dev->ext_filter_idx >= dev->hfdcan->Init.ExtFiltersNbr) {
        return;
    }
    if (HAL_FDCAN_ConfigFilter(dev->hfdcan, sFilterConfig) != HAL_OK) {
        return;
    }
    dev->ext_filter_idx++;
}

/**
 * @brief  注册标准范围滤波器 自动分配索引
 */
void FDCAN_StdRangeFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id_min, uint32_t id_max) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = dev->std_filter_idx;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_GetHalFifo(fifo);
    sFilterConfig.FilterID1 = id_min;
    sFilterConfig.FilterID2 = id_max;

    FDCAN_AddStdFilter(dev, &sFilterConfig);
}

/**
 * @brief  注册标准精确 ID 滤波器 自动分配索引
 */
void FDCAN_StdIdFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = dev->std_filter_idx;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_GetHalFifo(fifo);
    sFilterConfig.FilterID1 = id;
    sFilterConfig.FilterID2 = id;

    FDCAN_AddStdFilter(dev, &sFilterConfig);
}

/**
 * @brief  注册扩展范围滤波器 自动分配索引
 */
void FDCAN_ExtRangeFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id_min, uint32_t id_max) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = dev->ext_filter_idx;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_GetHalFifo(fifo);
    sFilterConfig.FilterID1 = id_min;
    sFilterConfig.FilterID2 = id_max;

    FDCAN_AddExtFilter(dev, &sFilterConfig);
}

/**
 * @brief  注册扩展精确 ID 滤波器 自动分配索引
 */
void FDCAN_ExtIdFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = dev->ext_filter_idx;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_GetHalFifo(fifo);
    sFilterConfig.FilterID1 = id;
    sFilterConfig.FilterID2 = id;

    FDCAN_AddExtFilter(dev, &sFilterConfig);
}

/**
 * @brief  启动 FDCAN 并启用目标 FIFO 中断
 * @note   未匹配帧进入目标 FIFO
 */
FDCAN_ErrorCode FDCAN_Start(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, FDCAN_IT_Line_Type interrupt_line) {
    if (!dev || !dev->hfdcan) {
        return FDCAN_ERR_INVALID_DEVICE;
    }

    if (dev->started != 0U) {
        return FDCAN_SetError(dev, FDCAN_ERR_NONE);
    }

    uint32_t Behavior = FDCAN_GetBehavior(fifo);
    uint32_t Act = FDCAN_GetActiveITs(fifo);

    if (HAL_FDCAN_ConfigGlobalFilter(dev->hfdcan, Behavior, Behavior, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) !=
        HAL_OK) {
        return FDCAN_SetError(dev, FDCAN_ERR_GLOBAL_FILTER_CONFIG_FAILED);
    }
    // 接收事件路由到目标中断线
    if (HAL_FDCAN_ConfigInterruptLines(dev->hfdcan, Act, (uint32_t)interrupt_line) != HAL_OK) {
        return FDCAN_SetError(dev, FDCAN_ERR_INTERRUPT_LINE_CONFIG_FAILED);
    }

    if (HAL_FDCAN_ActivateNotification(dev->hfdcan, Act, 0) != HAL_OK) {
        return FDCAN_SetError(dev, FDCAN_ERR_NOTIFICATION_ACTIVATE_FAILED);
    }

    if (HAL_FDCAN_Start(dev->hfdcan) != HAL_OK) {
        return FDCAN_SetError(dev, FDCAN_ERR_START_FAILED);
    }

    dev->active_fifo = fifo;
    dev->interrupt_line = interrupt_line;
    dev->started = 1U;
    dev->recovery_pending = 0U;
    return FDCAN_SetError(dev, FDCAN_ERR_NONE);
}

/**
 * @brief  在任务上下文中处理 Bus-Off 恢复请求。
 */
FDCAN_ErrorCode FDCAN_Service(FDCAN_Device* dev) {
    if (dev == NULL || dev->hfdcan == NULL) {
        return FDCAN_ERR_INVALID_DEVICE;
    }

    if (dev->recovery_pending == 0U) {
        return FDCAN_ERR_NONE;
    }

    (void)HAL_FDCAN_Stop(dev->hfdcan);
    dev->started = 0U;
    return FDCAN_Start(dev, dev->active_fifo, dev->interrupt_line);
}

/**
 * @brief  FDCAN 协议状态回调。中断中只记录状态，恢复由 FDCAN_Service 完成。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs) {
    FDCAN_Device* dev = FDCAN_GetDeviceByHandle(hfdcan);

    if (dev == NULL) {
        return;
    }

    dev->last_error_status = ErrorStatusITs;
    dev->error_count++;
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
        dev->bus_off_count++;
        dev->recovery_pending = 1U;
    }
}

/**
 * @brief  FDCAN HAL 错误回调。中断中不执行阻塞式恢复。
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef* hfdcan) {
    FDCAN_Device* dev = FDCAN_GetDeviceByHandle(hfdcan);

    if (dev == NULL) {
        return;
    }

    dev->last_hal_error = hfdcan->ErrorCode;
    dev->last_error = FDCAN_ERR_RUNTIME;
    dev->error_count++;
}

HAL_StatusTypeDef FDCAN_Transmit_Message(uint8_t bus_id, FDCAN_TxHeaderTypeDef* TxHeader, uint8_t* data) {
    FDCAN_Device* dev = FDCAN_GetDevice(bus_id);

    if (!dev || !dev->hfdcan || dev->started == 0U) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(dev->hfdcan, TxHeader, data);
    if (status != HAL_OK) {
        dev->tx_error_count++;
        dev->last_hal_error = dev->hfdcan->ErrorCode;
    }

    return status;
}

uint32_t FDCAN_LengthToDataLength(uint8_t length) {
    switch (length) {
        case 0:
            return FDCAN_DLC_BYTES_0;
        case 1:
            return FDCAN_DLC_BYTES_1;
        case 2:
            return FDCAN_DLC_BYTES_2;
        case 3:
            return FDCAN_DLC_BYTES_3;
        case 4:
            return FDCAN_DLC_BYTES_4;
        case 5:
            return FDCAN_DLC_BYTES_5;
        case 6:
            return FDCAN_DLC_BYTES_6;
        case 7:
            return FDCAN_DLC_BYTES_7;
        case 8:
        default:
            return FDCAN_DLC_BYTES_8;
    }
}

uint8_t FDCAN_DataLengthToLength(uint32_t data_length) {
    switch (data_length) {
        case FDCAN_DLC_BYTES_0:
            return 0U;
        case FDCAN_DLC_BYTES_1:
            return 1U;
        case FDCAN_DLC_BYTES_2:
            return 2U;
        case FDCAN_DLC_BYTES_3:
            return 3U;
        case FDCAN_DLC_BYTES_4:
            return 4U;
        case FDCAN_DLC_BYTES_5:
            return 5U;
        case FDCAN_DLC_BYTES_6:
            return 6U;
        case FDCAN_DLC_BYTES_7:
            return 7U;
        case FDCAN_DLC_BYTES_8:
            return 8U;
        default:
            return 8U;
    }
}
