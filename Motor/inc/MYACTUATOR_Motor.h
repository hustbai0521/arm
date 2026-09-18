/**
 * @file MYACTUATOR_Motor.h
 * @brief MYACTUATOR RMD-X4-36 V4 CAN motor driver.
 */
#ifndef MYACTUATOR_MOTOR_H_
#define MYACTUATOR_MOTOR_H_

#include "FDCAN_Basic.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MYACTUATOR_CAN_ID_MIN 1U
#define MYACTUATOR_CAN_ID_MAX 32U
#define MYACTUATOR_COMMAND_ID_BASE 0x140U
#define MYACTUATOR_REPLY_ID_BASE 0x240U
#define MYACTUATOR_MOTION_COMMAND_ID_BASE 0x400U
#define MYACTUATOR_MOTION_REPLY_ID_BASE 0x500U

#define MYACTUATOR_X4_36_TORQUE_CONSTANT 1.9f
#define MYACTUATOR_X4_36_RATED_TORQUE 10.5f
#define MYACTUATOR_X4_36_PEAK_TORQUE 34.0f

#define MYACTUATOR_MOTION_P_MIN (-12.5f)
#define MYACTUATOR_MOTION_P_MAX 12.5f
#define MYACTUATOR_MOTION_V_MIN (-45.0f)
#define MYACTUATOR_MOTION_V_MAX 45.0f
#define MYACTUATOR_MOTION_T_MIN (-24.0f)
#define MYACTUATOR_MOTION_T_MAX 24.0f
#define MYACTUATOR_MOTION_KP_MIN 0.0f
#define MYACTUATOR_MOTION_KP_MAX 500.0f
#define MYACTUATOR_MOTION_KD_MIN 0.0f
#define MYACTUATOR_MOTION_KD_MAX 5.0f

#define MYACTUATOR_FEEDBACK_LOST_LIMIT 1000U

typedef enum {
    MYACTUATOR_CTRL_STOP = 0,
    MYACTUATOR_CTRL_TORQUE,
    MYACTUATOR_CTRL_SPEED,
    MYACTUATOR_CTRL_POSITION,
    MYACTUATOR_CTRL_MOTION
} MYACTUATOR_ControlMode;

typedef enum {
    MYACTUATOR_CMD_SHUTDOWN = 0x80U,
    MYACTUATOR_CMD_STOP = 0x81U,
    MYACTUATOR_CMD_READ_MULTI_TURN_ANGLE = 0x92U,
    MYACTUATOR_CMD_READ_STATUS_1 = 0x9AU,
    MYACTUATOR_CMD_READ_STATUS_2 = 0x9CU,
    MYACTUATOR_CMD_TORQUE = 0xA1U,
    MYACTUATOR_CMD_SPEED = 0xA2U,
    MYACTUATOR_CMD_ABSOLUTE_POSITION = 0xA4U
} MYACTUATOR_Command;

typedef struct {
    uint8_t connected;
    uint8_t feedback_ready;
    uint16_t lost_count;
    uint32_t last_rx_count;
} MYACTUATOR_Flag;

typedef struct {
    uint8_t id;
    uint8_t can_bus;
    MYACTUATOR_ControlMode ctrl_mode;

    float Pos;       /* rad */
    float W;         /* rad/s */
    float T;         /* N.m */
    float K_P;
    float K_W;
    float max_speed; /* rad/s, position mode limit; 0 means firmware default */

    float rx_Pos;     /* rad */
    float rx_W;       /* rad/s */
    float rx_T;       /* N.m, calculated from iq */
    float rx_Current; /* A */
    float rx_Temp;    /* degree C */
    float rx_Voltage; /* V, after READ_STATUS_1 */
    uint16_t rx_Error;
    uint8_t last_command;

    MYACTUATOR_Flag flag;
    uint32_t tx_count;
    uint32_t rx_count;
} MYACTUATOR_Motor;

void MYACTUATOR_Motor_Init(MYACTUATOR_Motor* motor, uint8_t can_bus, uint8_t id);
void MYACTUATOR_Motor_ControlUpdate(MYACTUATOR_Motor* motor);
void MYACTUATOR_Motor_Transmit(MYACTUATOR_Motor* motor);
void MYACTUATOR_Motor_Shutdown(MYACTUATOR_Motor* motor);
void MYACTUATOR_Motor_Stop(MYACTUATOR_Motor* motor);
void MYACTUATOR_Motor_ReadStatus(MYACTUATOR_Motor* motor);
void MYACTUATOR_Motor_ReadMultiTurnAngle(MYACTUATOR_Motor* motor);
uint8_t MYACTUATOR_Motor_ProcessFrame(const FDCANFrame* frame, MYACTUATOR_Motor* motor);
uint8_t MYACTUATOR_Motor_RxHandler(const FDCANFrame* frame, void* context);

/*
 * 多实例接收注册示例：
 *   MYACTUATOR_Motor motor2;
 *   MYACTUATOR_Motor_Init(&motor2, 1U, 2U);
 *   FDCAN_RxDispatcher_Register(1U, MYACTUATOR_Motor_RxHandler, &motor2);
 *
 * 控制示例（位置单位 rad，速度 rad/s，力矩 N.m）：
 *   motor2.ctrl_mode = MYACTUATOR_CTRL_POSITION;
 *   motor2.Pos = 1.0f;
 *   motor2.max_speed = 2.0f;
 *   MYACTUATOR_Motor_Transmit(&motor2);
 */

#ifdef __cplusplus
}
#endif

#endif /* MYACTUATOR_MOTOR_H_ */
