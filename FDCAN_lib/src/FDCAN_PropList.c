#include "FDCAN_Proplist.h"

CANFunDict g_CAN1_Prop_Array[] = {{EXTID_NON_PROTOCOL, ExtID_Non_Protocol_FDCAN1},
                                  {STDID_NON_PROTOCOL, StdID_Non_Protocol_FDCAN1}};

// 当前只使用 FDCAN1，注释 FDCAN2 属性处理表。
// CANFunDict g_CAN2_Prop_Array[] = {
//     {NULL, NULL},
// };

const uint8_t g_CAN1_Prop_Count = sizeof(g_CAN1_Prop_Array) / sizeof(g_CAN1_Prop_Array[0]);
// 当前只使用 FDCAN1，注释 FDCAN2 属性处理表计数。
// const uint8_t g_CAN2_Prop_Count = sizeof(g_CAN2_Prop_Array) / sizeof(g_CAN2_Prop_Array[0]);
