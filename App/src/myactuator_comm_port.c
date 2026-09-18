#include "myactuator_comm_port.h"
#include <math.h>
#include <stddef.h>

MYACTUATOR_Motor myactuator_x4_36;
volatile FDCAN_RxDispatchStatus myactuator_register_status;
volatile MYACTUATOR_TestCommand myactuator_test_command = {
    .control_mode = MYACTUATOR_CTRL_POSITION,
    .id = MYACTUATOR_COMM_DEFAULT_ID,
};

/*
 * enabled=0 表示保持总线静默。仅在使能请求从 1 变为 0 时挂起一帧
 * STOP，避免未启用电机时由 1 ms 任务周期持续发送停止命令。
 */
static uint8_t myactuator_enabled_request;
static uint8_t myactuator_stop_pending;

FDCAN_RxDispatchStatus MYACTUATOR_Comm_Init(void) {
    myactuator_enabled_request = 0U;
    myactuator_stop_pending = 0U;
    MYACTUATOR_Motor_Init(&myactuator_x4_36, MYACTUATOR_COMM_CAN_BUS, MYACTUATOR_COMM_DEFAULT_ID);
    myactuator_register_status = FDCAN_RxDispatcher_Register(MYACTUATOR_COMM_CAN_BUS,
                                                             MYACTUATOR_Motor_RxHandler,
                                                             &myactuator_x4_36);
    if (myactuator_register_status == FDCAN_RX_DISPATCH_DUPLICATE) {
        myactuator_register_status = FDCAN_RX_DISPATCH_OK;
    }
    return myactuator_register_status;
}

HAL_StatusTypeDef MYACTUATOR_Comm_Configure(uint8_t id) {
    if (id < MYACTUATOR_CAN_ID_MIN || id > MYACTUATOR_CAN_ID_MAX ||
        myactuator_enabled_request != 0U) {
        return HAL_ERROR;
    }

    MYACTUATOR_Motor_Init(&myactuator_x4_36, MYACTUATOR_COMM_CAN_BUS, id);
    return HAL_OK;
}

void MYACTUATOR_Comm_SetEnabled(uint8_t enabled) {
    enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == myactuator_enabled_request) {
        return;
    }

    myactuator_enabled_request = enabled;
    if (enabled == 0U) {
        myactuator_x4_36.ctrl_mode = MYACTUATOR_CTRL_STOP;
        myactuator_stop_pending = 1U;
    } else {
        myactuator_stop_pending = 0U;
    }
}

uint8_t MYACTUATOR_Comm_GetEnabled(void) {
    return myactuator_enabled_request;
}

bool MYACTUATOR_Comm_TryReadCommand(MYACTUATOR_TestCommand* output) {
    MYACTUATOR_TestCommand snapshot;
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (output == NULL)
        return false;

    sequence_before = myactuator_test_command.sequence;
    if ((sequence_before & 1U) != 0U)
        return false;

    __DMB();
    snapshot.control_mode = myactuator_test_command.control_mode;
    snapshot.id = myactuator_test_command.id;
    snapshot.enabled = myactuator_test_command.enabled;
    snapshot.position = myactuator_test_command.position;
    snapshot.speed = myactuator_test_command.speed;
    snapshot.torque = myactuator_test_command.torque;
    snapshot.kp = myactuator_test_command.kp;
    snapshot.kd = myactuator_test_command.kd;
    __DMB();

    sequence_after = myactuator_test_command.sequence;
    if (sequence_before != sequence_after || (sequence_after & 1U) != 0U)
        return false;

    snapshot.sequence = sequence_after;
    *output = snapshot;
    return true;
}

void MYACTUATOR_Comm_ApplyCommand(const MYACTUATOR_TestCommand* command) {
    bool parameters_valid = false;

    if (command == NULL || command->control_mode > MYACTUATOR_CTRL_MOTION) {
        MYACTUATOR_Comm_SetEnabled(0U);
        return;
    }

    if (command->id != myactuator_x4_36.id) {
        if (MYACTUATOR_Comm_GetEnabled() != 0U) {
            MYACTUATOR_Comm_SetEnabled(0U);
            return;
        }
        if (command->enabled != 0U || MYACTUATOR_Comm_Configure(command->id) != HAL_OK)
            return;
    }

    if (command->enabled == 0U || command->control_mode == MYACTUATOR_CTRL_STOP) {
        MYACTUATOR_Comm_SetEnabled(0U);
        return;
    }

    switch (command->control_mode) {
        case MYACTUATOR_CTRL_TORQUE:
            parameters_valid = isfinite(command->torque) &&
                               fabsf(command->torque) <= 622.573f;
            break;
        case MYACTUATOR_CTRL_SPEED:
            parameters_valid = isfinite(command->speed) &&
                               fabsf(command->speed) <= 374000.0f;
            break;
        case MYACTUATOR_CTRL_POSITION:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               fabsf(command->position) <= 374000.0f &&
                               command->speed >= 0.0f &&
                               command->speed <= 1143.8f;
            break;
        case MYACTUATOR_CTRL_MOTION:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               isfinite(command->torque) &&
                               isfinite(command->kp) &&
                               isfinite(command->kd) &&
                               command->position >= MYACTUATOR_MOTION_P_MIN &&
                               command->position <= MYACTUATOR_MOTION_P_MAX &&
                               command->speed >= MYACTUATOR_MOTION_V_MIN &&
                               command->speed <= MYACTUATOR_MOTION_V_MAX &&
                               command->torque >= MYACTUATOR_MOTION_T_MIN &&
                               command->torque <= MYACTUATOR_MOTION_T_MAX &&
                               command->kp >= MYACTUATOR_MOTION_KP_MIN &&
                               command->kp <= MYACTUATOR_MOTION_KP_MAX &&
                               command->kd >= MYACTUATOR_MOTION_KD_MIN &&
                               command->kd <= MYACTUATOR_MOTION_KD_MAX;
            break;
        default:
            break;
    }

    if (!parameters_valid) {
        MYACTUATOR_Comm_SetEnabled(0U);
        return;
    }

    myactuator_x4_36.Pos = command->position;
    myactuator_x4_36.W = command->speed;
    myactuator_x4_36.T = command->torque;
    myactuator_x4_36.K_P = command->kp;
    myactuator_x4_36.K_W = command->kd;
    myactuator_x4_36.max_speed = command->speed;
    myactuator_x4_36.ctrl_mode = command->control_mode;
    MYACTUATOR_Comm_SetEnabled(1U);
}

void MYACTUATOR_Comm_ProcessAndTransmit(void) {
    if (myactuator_enabled_request == 0U) {
        if (myactuator_stop_pending != 0U) {
            MYACTUATOR_Motor_Stop(&myactuator_x4_36);
            myactuator_stop_pending = 0U;
        }
        return;
    }

    MYACTUATOR_Motor_Transmit(&myactuator_x4_36);
}
