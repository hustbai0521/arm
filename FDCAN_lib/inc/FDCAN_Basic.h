#ifndef FDCAN_BASIC_H_
#define FDCAN_BASIC_H_

#include "FDCAN_IDconf.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ID_1_4 1
#define ID_5_8 5

#define USE_CAN_1
// 当前只使用 FDCAN1，注释 FDCAN2 使能宏，避免启用未配置的 hfdcan2。
// #define USE_CAN_2

typedef struct
{
    uint32_t DesDeviceId : 8;
    uint32_t Property : 8;
    uint32_t SrcDeviceId : 8;
    uint32_t Priority : 4;
    uint32_t Permit : 1;
} EXT_ID_Typedef;

typedef union {
    uint32_t all;
    uint32_t StdID : 11;   // ID
    EXT_ID_Typedef ExtID;  // ID
} ID;

typedef union {
    int8_t chars[8];
    int16_t shorts[4];
    int32_t ints[2];
    int64_t longs[1];
    uint8_t uchars[8];
    uint16_t ushorts[4];
    uint32_t uints[2];
    uint64_t ulongs[1];
    float floats[2];
} FDCAN_Data;

typedef struct
{
    uint8_t canbus_id;  // CAN总线编号
    uint8_t IDE;        // IDE(是标准帧还是扩展帧)
    char isRemote;      // 是否为远程帧
    uint8_t Length;     // 数据长度
    FDCAN_Data Data;    // 实际数据
    ID Id;              // 帧ID
} FDCANFrame;

typedef struct
{
    uint16_t Prop;                 // 属性名称
    void (*Fun)(FDCANFrame* Frm);  // 此属性对应的处理函数
} CANFunDict;

typedef enum {
    CAN_RX_FIFO0 = 0,
    CAN_RX_FIFO1 = 1,
    CAN_RX_FIFO_BOTH = 2
} FDCAN_RX_FIFO_Type;

typedef enum {
    FDCAN_IT_LINE0 = FDCAN_INTERRUPT_LINE0,
    FDCAN_IT_LINE1 = FDCAN_INTERRUPT_LINE1
} FDCAN_IT_Line_Type;

typedef enum {
    FDCAN_ERR_NONE = 0,
    FDCAN_ERR_INVALID_DEVICE,
    FDCAN_ERR_FILTER_OVERFLOW,
    FDCAN_ERR_FILTER_CONFIG_FAILED,
    FDCAN_ERR_GLOBAL_FILTER_CONFIG_FAILED,
    FDCAN_ERR_START_FAILED,
    FDCAN_ERR_INTERRUPT_LINE_CONFIG_FAILED,
    FDCAN_ERR_NOTIFICATION_ACTIVATE_FAILED,
    FDCAN_ERR_TX_FAILED,
    FDCAN_ERR_RUNTIME
} FDCAN_ErrorCode;

// FDCAN 设备对象，面向对象方式封装相关状态与外设句柄
typedef struct
{
    FDCAN_HandleTypeDef* hfdcan;  // 底层外设句柄 (如 &hfdcan1)
    uint8_t bus_id;               // CAN总线编号 (1或2)
    uint32_t std_filter_idx;      // 当前已分配的标准帧滤波器索引
    uint32_t ext_filter_idx;      // 当前已分配的扩展帧滤波器索引
    FDCAN_RX_FIFO_Type active_fifo;
    FDCAN_IT_Line_Type interrupt_line;
    volatile uint8_t started;
    volatile uint8_t recovery_pending;
    volatile FDCAN_ErrorCode last_error;
    volatile uint32_t last_hal_error;
    volatile uint32_t last_error_status;
    volatile uint32_t error_count;
    volatile uint32_t bus_off_count;
    volatile uint32_t tx_error_count;
} FDCAN_Device;

// 将 can1 通过 extern 供全局其它文件调用
extern FDCAN_Device can1;
// 当前只使用 FDCAN1，注释 FDCAN2 设备声明。
// extern FDCAN_Device can2;
FDCAN_Device* FDCAN_GetDevice(uint8_t bus_id);

void FDCAN_StdRangeFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id_min, uint32_t id_max);
void FDCAN_StdIdFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id);
void FDCAN_ExtRangeFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id_min, uint32_t id_max);
void FDCAN_ExtIdFilter(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, uint32_t id);
FDCAN_ErrorCode FDCAN_Start(FDCAN_Device* dev, FDCAN_RX_FIFO_Type fifo, FDCAN_IT_Line_Type interrupt_line);
FDCAN_ErrorCode FDCAN_Service(FDCAN_Device* dev);

void Send_Frame_CAN(FDCANFrame* Frame_Send, int canx);
void DeQueueCanMessage(void);
uint32_t FDCAN_LengthToDataLength(uint8_t length);
uint8_t FDCAN_DataLengthToLength(uint32_t data_length);

#if defined(USE_CAN_1) || defined(USE_CAN_2)
HAL_StatusTypeDef FDCAN_Transmit_Message(uint8_t bus_id, FDCAN_TxHeaderTypeDef* TxHeader, uint8_t* data);
#endif

#endif
