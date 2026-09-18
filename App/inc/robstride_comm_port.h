#ifndef ROBSTRIDE_COMM_PORT_H_
#define ROBSTRIDE_COMM_PORT_H_

#include "FDCAN_RxDispatcher.h"
#include "RobStride_Motor.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 灵足电机与其他电机共用 FDCAN1，通过 29 位扩展 ID 区分协议。 */
#define ROBSTRIDE_COMM_CAN_BUS 1U
#define ROBSTRIDE_COMM_DEFAULT_ID 1U
/* O1/RS01 与 O5/EL05 共用扩展帧协议，仅 MIT 编解码量程不同。 */
#define ROBSTRIDE_COMM_DEFAULT_TYPE ROBSTRIDE_TYPE_RS01

/** 灵足电机控制命令邮箱；sequence 为偶数时表示一版完整命令。 */
typedef struct {
    uint32_t sequence;
    RobStrideControlMode control_mode;
    uint16_t id;
    uint8_t enabled;
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
} ROBSTRIDE_TestCommand;

/** 应用层使用的灵足单电机实例；具体型号由 ROBSTRIDE_COMM_DEFAULT_TYPE 决定。 */
extern RobStrideMotor robstride_motor;
extern volatile FDCAN_RxDispatchStatus robstride_register_status;
extern volatile ROBSTRIDE_TestCommand robstride_test_command;

/** 初始化灵足对象并将扩展帧解析器注册到通用 FDCAN 分发器。 */
FDCAN_RxDispatchStatus RobStrideComm_Init(void);

/** 仅在电机失能时重新配置型号和 ID。 */
HAL_StatusTypeDef RobStrideComm_Configure(RobStrideMotorType type, uint16_t id);

/** 设置应用层使能请求；失能时会向电机发送一次失能命令。 */
void RobStrideComm_SetEnabled(uint8_t enabled);
uint8_t RobStrideComm_GetEnabled(void);
bool RobStrideComm_TryReadCommand(ROBSTRIDE_TestCommand* output);
void RobStrideComm_ApplyCommand(const ROBSTRIDE_TestCommand* command);

/** 更新灵足控制量并发送一帧；由 MotorTask 周期调用。 */
void RobStrideComm_ProcessAndTransmit(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBSTRIDE_COMM_PORT_H_ */
