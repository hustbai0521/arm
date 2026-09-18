#include "robstride_comm_port.h"
#include <math.h>
#include <stddef.h>

RobStrideMotor robstride_motor;
volatile FDCAN_RxDispatchStatus robstride_register_status;
volatile ROBSTRIDE_TestCommand robstride_test_command = {
    .control_mode = ROBSTRIDE_CTRL_NATIVE_MIT,
    .id = ROBSTRIDE_COMM_DEFAULT_ID,
};

/*
 * 使能请求与电机反馈状态分开保存：电机掉线后仍可由周期发送
 * 重新使能，避免“等待反馈才发送、等待发送才反馈”的死锁。
 */
static uint8_t robstride_enabled_request;
static uint8_t robstride_disable_pending;

FDCAN_RxDispatchStatus RobStrideComm_Init(void) {
    robstride_enabled_request = 0U;
    robstride_disable_pending = 1U;
    RobStrideMotor_Init(&robstride_motor,
                        ROBSTRIDE_COMM_DEFAULT_TYPE,
                        ROBSTRIDE_COMM_CAN_BUS,
                        ROBSTRIDE_COMM_DEFAULT_ID);

    robstride_register_status =
        FDCAN_RxDispatcher_Register(ROBSTRIDE_COMM_CAN_BUS,
                                    RobStrideMotor_RxHandler,
                                    &robstride_motor);
    if (robstride_register_status == FDCAN_RX_DISPATCH_DUPLICATE)
        robstride_register_status = FDCAN_RX_DISPATCH_OK;

    return robstride_register_status;
}

HAL_StatusTypeDef RobStrideComm_Configure(RobStrideMotorType type,
                                          uint16_t id) {
    if ((type != ROBSTRIDE_TYPE_RS00 && type != ROBSTRIDE_TYPE_RS01 &&
         type != ROBSTRIDE_TYPE_RS05 && type != ROBSTRIDE_TYPE_EL05) ||
        id > ROBSTRIDE_MAX_MOTOR_ID || robstride_enabled_request != 0U)
        return HAL_ERROR;

    RobStrideMotor_Init(&robstride_motor, type,
                        ROBSTRIDE_COMM_CAN_BUS, id);
    robstride_disable_pending = 1U;
    return HAL_OK;
}

void RobStrideComm_SetEnabled(uint8_t enabled) {
    enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == robstride_enabled_request)
        return;

    robstride_enabled_request = enabled;
    if (enabled == 0U) {
        robstride_motor.ctrl_mode = ROBSTRIDE_CTRL_STOP;
        /* 下次使能必须先等到电机重新进入 RUN，不直接发控制量。 */
        robstride_motor.Data.rx_mode_state = ROBSTRIDE_STATE_RESET;
        robstride_disable_pending = 1U;
    } else {
        robstride_disable_pending = 0U;
    }
    /*
     * 使能帧由 ProcessAndTransmit 发送。这样同一周期不会先发一帧
     * Enable，紧接着又因尚未收到 RUN 反馈而重复发送。
     */
}

uint8_t RobStrideComm_GetEnabled(void) {
    return robstride_enabled_request;
}

bool RobStrideComm_TryReadCommand(ROBSTRIDE_TestCommand* output) {
    ROBSTRIDE_TestCommand snapshot;
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (output == NULL)
        return false;

    sequence_before = robstride_test_command.sequence;
    if ((sequence_before & 1U) != 0U)
        return false;

    __DMB();
    snapshot.control_mode = robstride_test_command.control_mode;
    snapshot.id = robstride_test_command.id;
    snapshot.enabled = robstride_test_command.enabled;
    snapshot.position = robstride_test_command.position;
    snapshot.speed = robstride_test_command.speed;
    snapshot.torque = robstride_test_command.torque;
    snapshot.kp = robstride_test_command.kp;
    snapshot.kd = robstride_test_command.kd;
    __DMB();

    sequence_after = robstride_test_command.sequence;
    if (sequence_before != sequence_after || (sequence_after & 1U) != 0U)
        return false;

    snapshot.sequence = sequence_after;
    *output = snapshot;
    return true;
}

void RobStrideComm_ApplyCommand(const ROBSTRIDE_TestCommand* command) {
    const RobStrideMotorLimit* limit = &robstride_motor.Data.limit;
    bool parameters_valid = false;

    if (command == NULL || command->id > ROBSTRIDE_MAX_MOTOR_ID ||
        command->control_mode > ROBSTRIDE_CTRL_SPD) {
        RobStrideComm_SetEnabled(0U);
        robstride_motor.ctrl_mode = ROBSTRIDE_CTRL_STOP;
        return;
    }

    if (command->id != robstride_motor.id) {
        RobStrideComm_SetEnabled(0U);
        robstride_motor.ctrl_mode = ROBSTRIDE_CTRL_STOP;
        if (command->enabled != 0U ||
            RobStrideComm_Configure(ROBSTRIDE_COMM_DEFAULT_TYPE,
                                    command->id) != HAL_OK)
            return;
        limit = &robstride_motor.Data.limit;
    }

    if (command->enabled == 0U || command->control_mode == ROBSTRIDE_CTRL_STOP) {
        RobStrideComm_SetEnabled(0U);
        robstride_motor.ctrl_mode = ROBSTRIDE_CTRL_STOP;
        return;
    }

    switch (command->control_mode) {
        case ROBSTRIDE_CTRL_NATIVE_MIT:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               isfinite(command->torque) &&
                               isfinite(command->kp) &&
                               isfinite(command->kd) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max &&
                               command->speed >= limit->v_min &&
                               command->speed <= limit->v_max &&
                               command->torque >= limit->t_min &&
                               command->torque <= limit->t_max &&
                               command->kp >= limit->kp_min &&
                               command->kp <= limit->kp_max &&
                               command->kd >= limit->kd_min &&
                               command->kd <= limit->kd_max;
            break;
        case ROBSTRIDE_CTRL_POS:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->kd) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max &&
                               command->kd >= limit->kd_min &&
                               command->kd <= limit->kd_max;
            break;
        case ROBSTRIDE_CTRL_SPD:
            parameters_valid = isfinite(command->speed) &&
                               command->speed >= limit->v_min &&
                               command->speed <= limit->v_max;
            break;
        default:
            break;
    }

    if (!parameters_valid) {
        RobStrideComm_SetEnabled(0U);
        robstride_motor.ctrl_mode = ROBSTRIDE_CTRL_STOP;
        return;
    }

    robstride_motor.Pos = command->position;
    robstride_motor.W = command->speed;
    robstride_motor.T = command->torque;
    robstride_motor.K_P = command->kp;
    robstride_motor.K_W = command->kd;
    robstride_motor.ctrl_mode = command->control_mode;
    RobStrideComm_SetEnabled(1U);
}

void RobStrideComm_ProcessAndTransmit(void) {
    if (robstride_enabled_request == 0U) {
        /* 上电或刚切换到 disabled 时发送一次，后续保持总线安静。 */
        if (robstride_disable_pending != 0U) {
            RobStrideMotor_Disable(&robstride_motor);
            robstride_disable_pending = 0U;
        }
        return;
    }

    /* 运行后掉线时重新走 Enable -> RUN 握手，不持续发无效控制帧。 */
    if (robstride_motor.rx_count != 0U &&
        robstride_motor.flag.connected == 0U)
        robstride_motor.Data.rx_mode_state = ROBSTRIDE_STATE_RESET;

    RobStrideMotor_Control_Update(&robstride_motor);
    RobStrideMotor_Transmit(&robstride_motor);
}
