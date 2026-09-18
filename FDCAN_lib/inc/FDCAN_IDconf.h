/**
 ******************************************************************************
 * @file    RobotCAN_IDconf.h
 * @author  Robocon
 * @brief   本代码提供了机器人上所有驱动的CAN通信ID定义及本设备ID定义:
 *           - 主控ID定义
 *           - 底盘设备ID定义
 *           - 上层设备ID定义
 *           - 本设备CAN通信ID及滤波器定义
 *  @verbatim
 *          根据每年机器人不同进行修改
 *  @endverbatim
 ******************************************************************************
 */
#ifndef ROBOTCAN_IDCONF_H_
#define ROBOTCAN_IDCONF_H_

/******************************整车设备ID定义（团队协议通信部分）*****************************/
// INTERNAL DEVICE ID
#define LOCATE_ID 0xff
// #define MASTER_ID                   0xfe
#define LOCATE_SECOND_ID 0x01
#define MASTER_SECOND_ID 0x02

// CHASSIS
#define CHASSIS_BROADCAST 0x00
#define CHASSIS_TRAIN1 0x01
#define CHASSIS_TRAIN2 0x02
#define CHASSIS_TRAIN3 0x03
#define CHASSIS_TRAIN4 0x04
#define CHASSIS_BROADCAST_ID (CHASSIS_BROADCAST)
#define CHASSIS_TRAIN1_ID (CHASSIS_BROADCAST_ID + CHASSIS_TRAIN1)
#define CHASSIS_TRAIN2_ID (CHASSIS_BROADCAST_ID + CHASSIS_TRAIN2)
#define CHASSIS_TRAIN3_ID (CHASSIS_BROADCAST_ID + CHASSIS_TRAIN3)
#define CHASSIS_TRAIN4_ID (CHASSIS_BROADCAST_ID + CHASSIS_TRAIN4)

// UP MACHINE
// ID:1xxx(B)  //virtual sub net
#define UP_BROADCAST 0x80
#define UP_PUSH_MECHANISM 0x01
#define UP_MACHINE_BROADCAST_ID (UP_BROADCAST)
#define UP_PUSH_MECHANISM_ID (UP_MACHINE_BROADCAST_ID + UP_PUSH_MECHANISM)

/******************************整车设备ID定义（非团队协议通信部分）*****************************/
// CHASSIS
#define CHASSIS_WHEEL_MOTOR1_ID 1
#define CHASSIS_WHEEL_MOTOR2_ID 2
#define CHASSIS_WHEEL_MOTOR3_ID 3
#define CHASSIS_HELM_MOTOR1_ID 11
#define CHASSIS_HELM_MOTOR2_ID 12
#define CHASSIS_HELM_MOTOR3_ID 13

/******************************本设备ID定义*****************************/
#define DEVICE_ID LOCATE_ID
#define BROADCAST_ID CHASSIS_BROADCAST_ID

#if (DEVICE_ID == LOCATE_ID)
#define DEVICE_SECOND_ID LOCATE_SECOND_ID
#elif (DEVICE_ID == MASTER_ID)
#define DEVICE_SECOND_ID MASTER_SECOND_ID
#else
#define SLAVER_DEVICE
#define DEVICE_SECOND_ID BROADCAST_ID
#endif

#define FILTER_ID DEVICE_ID
#define FILTER_SECOND_ID DEVICE_SECOND_ID
#define FILTER_MASK 0x00000000

// 编码器地址
#define ENCODER_ID 0x01

#endif /*ROBOT_CAN_CONF_H_*/
