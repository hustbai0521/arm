#ifndef MYACTUATOR_COMM_PORT_H_
#define MYACTUATOR_COMM_PORT_H_

#include "FDCAN_RxDispatcher.h"
#include "MYACTUATOR_Motor.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MYACTUATOR_COMM_CAN_BUS 1U
#define MYACTUATOR_COMM_DEFAULT_ID 1U

/** 脉塔电机控制命令邮箱；sequence 为偶数时表示一版完整命令。 */
typedef struct {
    uint32_t sequence;
    MYACTUATOR_ControlMode control_mode;
    uint8_t id;
    uint8_t enabled;
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
} MYACTUATOR_TestCommand;

extern MYACTUATOR_Motor myactuator_x4_36;
extern volatile FDCAN_RxDispatchStatus myactuator_register_status;
extern volatile MYACTUATOR_TestCommand myactuator_test_command;

FDCAN_RxDispatchStatus MYACTUATOR_Comm_Init(void);
HAL_StatusTypeDef MYACTUATOR_Comm_Configure(uint8_t id);
void MYACTUATOR_Comm_SetEnabled(uint8_t enabled);
uint8_t MYACTUATOR_Comm_GetEnabled(void);
bool MYACTUATOR_Comm_TryReadCommand(MYACTUATOR_TestCommand* output);
void MYACTUATOR_Comm_ApplyCommand(const MYACTUATOR_TestCommand* command);
void MYACTUATOR_Comm_ProcessAndTransmit(void);

#ifdef __cplusplus
}
#endif

#endif /* MYACTUATOR_COMM_PORT_H_ */
