#ifndef DAMIAO_COMM_PORT_H_
#define DAMIAO_COMM_PORT_H_

#include "DM_Motor.h"
#include "FDCAN_Basic.h"
#include "FDCAN_RxDispatcher.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DAMIAO_COMM_CAN_BUS 1U
#define DAMIAO_COMM_DEFAULT_ESC_ID 0x01U
#define DAMIAO_COMM_DEFAULT_MASTER_ID 0x021U
#ifndef DAMIAO_COMM_DEFAULT_MOTOR_TYPE
#define DAMIAO_COMM_DEFAULT_MOTOR_TYPE DAMIAO_MOTOR_TYPE_DM_J4340_2EC
#endif
#define DAMIAO_COMM_ENABLE_RETRY_PERIOD 100U

/** 达妙电机控制命令邮箱；sequence 为偶数时表示一版完整命令。 */
typedef struct {
    uint32_t sequence;
    DamiaoMotorType motor_type;
    DamiaoMotorControlMode control_mode;
    uint16_t esc_id;
    uint16_t master_id;
    uint8_t enabled;
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
    float current;
} DAMIAO_TestCommand;

/* FDCAN1 上的通用达妙电机对象；型号由 motor.type 选择。 */
extern DamiaoMotor damiao_motor;
extern volatile DAMIAO_TestCommand damiao_test_command;

extern volatile FDCAN_ErrorCode damiao_fdcan_start_error;

/* 兼容原有 Watch 名称；统计现已属于通用 FDCAN 接收分发层。 */
#define damiao_fdcan_rx_callback_count (fdcan_rx_dispatcher_diagnostics.callback_count)
#define damiao_fdcan_rx_error_count (fdcan_rx_dispatcher_diagnostics.read_error_count)
#define damiao_fdcan_rx_fifo_lost_count (fdcan_rx_dispatcher_diagnostics.fifo_lost_count)
#define damiao_fdcan_rx_fifo_full_count (fdcan_rx_dispatcher_diagnostics.fifo_full_count)
#define damiao_fdcan_ignored_frame_count (fdcan_rx_dispatcher_diagnostics.ignored_count)
#define damiao_fdcan_last_rx_its (fdcan_rx_dispatcher_diagnostics.last_interrupt_flags)

FDCAN_ErrorCode DamiaoComm_Init(void);
HAL_StatusTypeDef DamiaoComm_Configure(DamiaoMotorType type,
                                      uint16_t esc_id,
                                      uint16_t master_id);
void DamiaoComm_SetEnabled(uint8_t enabled);
uint8_t DamiaoComm_GetEnabled(void);
bool DamiaoComm_TryReadCommand(DAMIAO_TestCommand* output);
void DamiaoComm_ApplyCommand(const DAMIAO_TestCommand* command);
void DamiaoComm_ProcessAndTransmit(void);

#ifdef __cplusplus
}
#endif

#endif /* DAMIAO_COMM_PORT_H_ */
