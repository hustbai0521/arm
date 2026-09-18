#ifndef FDCAN_PROPLIST_
#define FDCAN_PROPLIST_

#include "FDCAN_Basic.h"
#include "fdcan.h"

extern CANFunDict g_CAN1_Prop_Array[];

// 当前只使用 FDCAN1，注释 FDCAN2 属性处理表声明。
// extern CANFunDict g_CAN2_Prop_Array[];

extern const uint8_t g_CAN1_Prop_Count;

// 当前只使用 FDCAN1，注释 FDCAN2 属性处理表计数声明。
// extern const uint8_t g_CAN2_Prop_Count;

/* CAN1 part */

/******************************LOCATE-DRIVER,CAN1*****************************/

/******************************prop属性值，8位*********************************/
#define EXTID_NON_PROTOCOL 0x00
#define STDID_NON_PROTOCOL 0x01

/******************************prio优先级，8位*********************************/

/* CAN2 part */

/******************************LOCATE-MASTER,CAN2****************************/

/******************************prop属性值，8位*********************************/
#define READANGLE 0x01
#define SETID 0x02
#define SETBAUNDRATE 0x03
#define SETMODE 0x04
#define SETBACKTIME 0x05
#define SETZERO 0x06
#define SETDIR 0x07
#define READVELOCITY 0x0A
#define SETSAMPLE 0x0B
#define SETMIDDLE 0x0C
#define SETPOSITION 0x0D
#define SETPOSITION_FIVE 0x0F

/******************************prio优先级，8位*********************************/

/******************************接收函数*********************************/

void ExtID_Non_Protocol_FDCAN1(FDCANFrame*);
void StdID_Non_Protocol_FDCAN1(FDCANFrame*);
// 当前只使用 FDCAN1，注释 FDCAN2 接收处理函数声明。
// void ExtID_Non_Protocol_FDCAN2(FDCANFrame*);
// void StdID_Non_Protocol_FDCAN2(FDCANFrame*);

/******************************发送函数*********************************/

#endif
