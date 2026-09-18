/**
 ******************************************************************************
 * @file    RobotCAN_ExecuteLib.c
 * @author  Robocon
 * @brief   本代码提供了CAN消息处理函数的定义:
 *           - CAN1，CAN2非团队协议扩展帧消息处理函数
 *           - 其他自定义团队协议扩展帧消息处理函数
 *  @verbatim
 *          根据需求添加相应的消息处理函数，添加后更新相应的CAN_PropList.c即可
 *  @endverbatim
 ******************************************************************************
 */
#include "FDCAN_Basic.h"
#include "FDCAN_Proplist.h"

/*******************************CAN1接收函数***********************************/
/**
 * @brief  CAN1非团队协议扩展帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
void ExtID_Non_Protocol_FDCAN1(FDCANFrame* Frame) {
}

/**
 * @brief  CAN1非团队协议标准帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
void StdID_Non_Protocol_FDCAN1(FDCANFrame* Frame) {
}

/*******************************CAN2接收函数***********************************/

/**
 * @brief  CAN2非团队协议扩展帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
// 当前只使用 FDCAN1，注释 FDCAN2 扩展帧接收处理函数。
// void ExtID_Non_Protocol_FDCAN2(FDCANFrame* Frame) {
// }

/**
 * @brief  CAN2非团队协议标准帧消息处理函数
 * @param  数据帧地址
 * @retval 无
 */
// 当前只使用 FDCAN1，注释 FDCAN2 标准帧接收处理函数。
// void StdID_Non_Protocol_FDCAN2(FDCANFrame* Frame) {
// }
