#include "motor_comm_port.h"

#include "usart.h"
#include <math.h>
#include <stddef.h>

UnitreeMotor unitree_a1_motor;
COM_UART_Port unitree_uart_port;
volatile UNITREE_A1_TestCommand unitree_a1_test_command = {
    .control_mode = UNITREE_CTRL_NATIVE,
    .id = UNITREE_COMM_DEFAULT_ID,
};

/* 串口模块按完整接收块回调；协议层仍以流状态机逐字节解析和重同步。 */
static void MotorComm_OnUnitreeData(const uint8_t* data, uint16_t length, void* context) {
    UnitreeMotor* motor = (UnitreeMotor*)context;
    uint16_t i;

    if (data == NULL || motor == NULL || motor->flag.fbk_ready_flag != 0U) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        Process_Unitree_Motor_Byte(motor, data[i]);
    }

    /* IDLE 已给出一批数据的物理边界；残留半帧不能带到下一次反馈。 */
    if (motor->flag.fbk_ready_flag == 0U &&
        (length != motor->param_scale.rx_msg_len ||
         motor->fbk_rx_index != 0U || motor->flag.rx_ignore_size != 0U)) {
        motor->rx_error = UNITREE_RX_ERROR_INCOMPLETE;
        motor->fbk_rx_index = 0U;
        motor->flag.rx_ignore_size = 0U;
    }
}

static HAL_StatusTypeDef MotorComm_SetReceiveLength(void) {
    return COM_UART_SetReceiveLength(&unitree_uart_port, COM_UART_RX_BUFFER_SIZE);
}

/* 兼容任务直接配置电机、尚未调用 MotorComm_Init 的启动路径。 */
static HAL_StatusTypeDef MotorComm_EnsureUnitreePort(void) {
    if (unitree_uart_port.huart != NULL) {
        return HAL_OK;
    }
    return COM_UART_Init(&unitree_uart_port, &huart1,
                         MotorComm_OnUnitreeData, &unitree_a1_motor);
}

/* 默认创建 A1、ID 1 电机，保持停止并启动 USART1 Receive-to-IDLE 中断接收。 */
void MotorComm_Init(void) {
    (void)COM_UART_StopReceive(&unitree_uart_port);
    UnitreeMotor_Init(&unitree_a1_motor, UNITREE_MOTOR_TYPE_A1, UNITREE_COMM_DEFAULT_ID);
    unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
    if (COM_UART_Init(&unitree_uart_port, &huart1,
                      MotorComm_OnUnitreeData, &unitree_a1_motor) == HAL_OK &&
        MotorComm_SetReceiveLength() == HAL_OK) {
        (void)MotorComm_StartUnitreeReceive();
    }
}

HAL_StatusTypeDef MotorComm_StartUnitreeReceive(void) {
    HAL_StatusTypeDef status = MotorComm_EnsureUnitreePort();
    if (status != HAL_OK) {
        return status;
    }
    return COM_UART_StartReceive(&unitree_uart_port);
}

/* 配置前停止接收；重新初始化后电机保持停止状态。 */
HAL_StatusTypeDef MotorComm_ConfigureUnitree(UnitreeMotorType type, uint8_t id) {
    HAL_StatusTypeDef status;
    if ((type != UNITREE_MOTOR_TYPE_GO1 && type != UNITREE_MOTOR_TYPE_A1) || id > UNITREE_MOTOR_ID_MAX) {
        return HAL_ERROR;
    }

    status = MotorComm_EnsureUnitreePort();
    if (status != HAL_OK) {
        return status;
    }
    status = COM_UART_StopReceive(&unitree_uart_port);
    if (status != HAL_OK) {
        return status;
    }
    UnitreeMotor_Init(&unitree_a1_motor, type, id);
    unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
    status = MotorComm_SetReceiveLength();
    if (status != HAL_OK) {
        return status;
    }
    return MotorComm_StartUnitreeReceive();
}

bool MotorComm_TryReadUnitreeCommand(UNITREE_A1_TestCommand* output) {
    UNITREE_A1_TestCommand snapshot;
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (output == NULL)
        return false;

    sequence_before = unitree_a1_test_command.sequence;
    if ((sequence_before & 1U) != 0U)
        return false;

    __DMB();
    snapshot.control_mode = unitree_a1_test_command.control_mode;
    snapshot.id = unitree_a1_test_command.id;
    snapshot.enabled = unitree_a1_test_command.enabled;
    snapshot.position = unitree_a1_test_command.position;
    snapshot.speed = unitree_a1_test_command.speed;
    snapshot.torque = unitree_a1_test_command.torque;
    snapshot.kp = unitree_a1_test_command.kp;
    snapshot.kw = unitree_a1_test_command.kw;
    __DMB();

    sequence_after = unitree_a1_test_command.sequence;
    if (sequence_before != sequence_after || (sequence_after & 1U) != 0U)
        return false;

    snapshot.sequence = sequence_after;
    *output = snapshot;
    return true;
}

void MotorComm_ApplyUnitreeCommand(const UNITREE_A1_TestCommand* command) {
    bool parameters_valid = false;

    if (command == NULL || command->id >= UNITREE_MOTOR_ID_MAX ||
        command->control_mode > UNITREE_CTRL_POS_SPD) {
        unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
        return;
    }

    if (command->id != unitree_a1_motor.id) {
        if (command->enabled != 0U ||
            MotorComm_ConfigureUnitree(UNITREE_MOTOR_TYPE_A1, command->id) != HAL_OK) {
            unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
            return;
        }
    }

    if (command->enabled == 0U || command->control_mode == UNITREE_CTRL_STOP) {
        unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
        return;
    }

    switch (command->control_mode) {
        case UNITREE_CTRL_NATIVE:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               isfinite(command->torque) &&
                               isfinite(command->kp) &&
                               isfinite(command->kw) &&
                               fabsf(command->position) <= 800000.0f &&
                               fabsf(command->speed) <= UNITREE_A1_LIMIT_SPEED &&
                               fabsf(command->torque) <= UNITREE_A1_LIMIT_TORQUE &&
                               command->kp >= 0.0f && command->kp <= 15.999f &&
                               command->kw >= 0.0f && command->kw <= 25.6f;
            break;
        case UNITREE_CTRL_POS:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->kw) &&
                               fabsf(command->position) <= 800000.0f &&
                               command->kw >= 0.0f && command->kw <= 25.6f;
            break;
        case UNITREE_CTRL_POS_SPD:
            parameters_valid = isfinite(command->position) &&
                               fabsf(command->position) <= 800000.0f;
            break;
        default:
            break;
    }

    if (!parameters_valid) {
        unitree_a1_motor.ctrl_mode = UNITREE_CTRL_STOP;
        return;
    }

    unitree_a1_motor.Pos = command->position;
    unitree_a1_motor.W = command->speed;
    unitree_a1_motor.T = command->torque;
    unitree_a1_motor.K_P = command->kp;
    unitree_a1_motor.K_W = command->kw;
    unitree_a1_motor.ctrl_mode = command->control_mode;
}

/* 由电机任务周期调用，更新控制状态并发送协议帧。 */
void MotorComm_ProcessAndTransmit(void) {
    /* 错误回调重装偶发失败时，任务侧在下一周期自动恢复接收。 */
    (void)COM_UART_EnsureReceive(&unitree_uart_port);
    UnitreeMotor_Control_Update(&unitree_a1_motor);
    UnitreeMotor_Transmit_DMA(&unitree_a1_motor, &huart1);
}
