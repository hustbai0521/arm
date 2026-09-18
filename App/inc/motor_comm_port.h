#ifndef MOTOR_COMM_PORT_H_
#define MOTOR_COMM_PORT_H_

#include "Unitree_Motor.h"
#include "COM_UART.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UNITREE_COMM_DEFAULT_ID 1U

/** 宇树 A1 控制命令邮箱；sequence 为偶数时表示一版完整命令。 */
typedef struct {
    uint32_t sequence;
    UnitreeControlMode control_mode;
    uint8_t id;
    uint8_t enabled;
    float position;
    float speed;
    float torque;
    float kp;
    float kw;
} UNITREE_A1_TestCommand;

/* 默认使用 USART1、A1 电机和 ID 1；上电时保持停止状态。 */
extern UnitreeMotor unitree_a1_motor;
extern volatile UNITREE_A1_TestCommand unitree_a1_test_command;

/* 4.8 Mbps 链路诊断数据，记录 HAL_UART_ERROR_* 位掩码。
 * 保留原名称的源码访问；调试器可直接观察 unitree_uart_port 的成员。 */
extern COM_UART_Port unitree_uart_port;
#define unitree_uart_error_count (unitree_uart_port.error_count)
#define unitree_uart_last_error (unitree_uart_port.last_error)
#define unitree_uart_restart_failure_count (unitree_uart_port.restart_failure_count)
#define unitree_uart_rx_block_count (unitree_uart_port.rx_block_count)
#define unitree_uart_overrun_error_count (unitree_uart_port.overrun_error_count)
#define unitree_uart_frame_error_count (unitree_uart_port.frame_error_count)
#define unitree_uart_noise_error_count (unitree_uart_port.noise_error_count)
#define unitree_uart_parity_error_count (unitree_uart_port.parity_error_count)

void MotorComm_Init(void);
HAL_StatusTypeDef MotorComm_StartUnitreeReceive(void);
HAL_StatusTypeDef MotorComm_ConfigureUnitree(UnitreeMotorType type, uint8_t id);
bool MotorComm_TryReadUnitreeCommand(UNITREE_A1_TestCommand* output);
void MotorComm_ApplyUnitreeCommand(const UNITREE_A1_TestCommand* command);
void MotorComm_ProcessAndTransmit(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_COMM_PORT_H_ */
