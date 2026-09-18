#include "damiao_comm_port.h"

#include "FDCAN_RxDispatcher.h"
#include "fdcan.h"
#include <math.h>
#include <stddef.h>

DamiaoMotor damiao_motor = {
    .id = DAMIAO_COMM_DEFAULT_ESC_ID,
    .mst_id = DAMIAO_COMM_DEFAULT_MASTER_ID,
    .can_bus = DAMIAO_COMM_CAN_BUS,
    .type = DAMIAO_COMM_DEFAULT_MOTOR_TYPE,
    .ctrl_mode = DAMIAO_CTRL_STOP,
};

volatile DAMIAO_TestCommand damiao_test_command = {
    .motor_type = DAMIAO_COMM_DEFAULT_MOTOR_TYPE,
    .control_mode = DAMIAO_CTRL_MIT,
    .esc_id = DAMIAO_COMM_DEFAULT_ESC_ID,
    .master_id = DAMIAO_COMM_DEFAULT_MASTER_ID,
};

volatile FDCAN_ErrorCode damiao_fdcan_start_error;

static uint8_t damiao_enabled_request;
static uint16_t damiao_enable_retry_count;

FDCAN_ErrorCode DamiaoComm_Init(void) {
    FDCAN_RxDispatchStatus register_status;
    DamiaoMotorType selected_type = damiao_motor.type;

    damiao_enabled_request = 0U;
    damiao_enable_retry_count = 0U;

    if (DamiaoMotor_IsTypeSupported(selected_type) == 0U) {
        damiao_fdcan_start_error = FDCAN_ERR_INVALID_DEVICE;
        return damiao_fdcan_start_error;
    }

    DamiaoMotor_Init(&damiao_motor,
                     selected_type,
                     DAMIAO_COMM_CAN_BUS,
                     DAMIAO_COMM_DEFAULT_ESC_ID);
    damiao_motor.mst_id = DAMIAO_COMM_DEFAULT_MASTER_ID;
    damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;

    register_status = FDCAN_RxDispatcher_Register(DAMIAO_COMM_CAN_BUS,
                                                  DamiaoMotor_RxHandler,
                                                  &damiao_motor);
    if (register_status != FDCAN_RX_DISPATCH_OK && register_status != FDCAN_RX_DISPATCH_DUPLICATE) {
        damiao_fdcan_start_error = FDCAN_ERR_RUNTIME;
        return damiao_fdcan_start_error;
    }

    /*
     * 所有共线协议（脉塔、达妙、灵足）都注册完成后，再由 main
     * 统一启动 FDCAN。避免灵足处理器尚未注册时就收到扩展帧。
     */
    damiao_fdcan_start_error = FDCAN_ERR_NONE;
    return damiao_fdcan_start_error;
}

HAL_StatusTypeDef DamiaoComm_Configure(DamiaoMotorType type,
                                      uint16_t esc_id,
                                      uint16_t master_id) {
    if (DamiaoMotor_IsTypeSupported(type) == 0U || esc_id > 0x0FU ||
        master_id > 0x7FFU) {
        return HAL_ERROR;
    }

    DamiaoComm_SetEnabled(0U);
    DamiaoMotor_Init(&damiao_motor,
                     type,
                     DAMIAO_COMM_CAN_BUS,
                     esc_id);
    damiao_motor.mst_id = master_id;
    damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
    damiao_enable_retry_count = 0U;
    return HAL_OK;
}

void DamiaoComm_SetEnabled(uint8_t enabled) {
    enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == damiao_enabled_request) {
        return;
    }

    damiao_enabled_request = enabled;
    damiao_enable_retry_count = 0U;
    if (enabled != 0U) {
        DamiaoMotor_Enable(&damiao_motor);
        damiao_enable_retry_count = DAMIAO_COMM_ENABLE_RETRY_PERIOD;
    } else {
        DamiaoMotor_Disable(&damiao_motor);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
    }
}

uint8_t DamiaoComm_GetEnabled(void) {
    return damiao_enabled_request;
}

bool DamiaoComm_TryReadCommand(DAMIAO_TestCommand* output) {
    DAMIAO_TestCommand snapshot;
    uint32_t sequence_before;
    uint32_t sequence_after;

    if (output == NULL)
        return false;

    sequence_before = damiao_test_command.sequence;
    if ((sequence_before & 1U) != 0U)
        return false;

    __DMB();
    snapshot.motor_type = damiao_test_command.motor_type;
    snapshot.control_mode = damiao_test_command.control_mode;
    snapshot.esc_id = damiao_test_command.esc_id;
    snapshot.master_id = damiao_test_command.master_id;
    snapshot.enabled = damiao_test_command.enabled;
    snapshot.position = damiao_test_command.position;
    snapshot.speed = damiao_test_command.speed;
    snapshot.torque = damiao_test_command.torque;
    snapshot.kp = damiao_test_command.kp;
    snapshot.kd = damiao_test_command.kd;
    snapshot.current = damiao_test_command.current;
    __DMB();

    sequence_after = damiao_test_command.sequence;
    if (sequence_before != sequence_after || (sequence_after & 1U) != 0U)
        return false;

    snapshot.sequence = sequence_after;
    *output = snapshot;
    return true;
}

void DamiaoComm_ApplyCommand(const DAMIAO_TestCommand* command) {
    const DamiaoMotorLimit* limit;
    bool parameters_valid = false;

    if (command == NULL || DamiaoMotor_IsTypeSupported(command->motor_type) == 0U ||
        command->esc_id > 0x0FU || command->master_id > 0x7FFU) {
        DamiaoComm_SetEnabled(0U);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
        return;
    }

    if (command->motor_type != damiao_motor.type ||
        command->esc_id != damiao_motor.id ||
        command->master_id != damiao_motor.mst_id) {
        DamiaoComm_SetEnabled(0U);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
        if (command->enabled != 0U)
            return;
        if (DamiaoComm_Configure(command->motor_type,
                                 command->esc_id,
                                 command->master_id) != HAL_OK)
            return;
    }

    limit = &damiao_motor.limit;
    if (command->control_mode > DAMIAO_CTRL_PSI ||
        DamiaoMotor_IsControlModeSupported(&damiao_motor,
                                           command->control_mode) == 0U) {
        DamiaoComm_SetEnabled(0U);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
        return;
    }

    if (command->enabled == 0U || command->control_mode == DAMIAO_CTRL_STOP) {
        DamiaoComm_SetEnabled(0U);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
        return;
    }

    switch (command->control_mode) {
        case DAMIAO_CTRL_MIT:
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
        case DAMIAO_CTRL_MIT_POS:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->kd) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max &&
                               command->kd >= limit->kd_min &&
                               command->kd <= limit->kd_max;
            break;
        case DAMIAO_CTRL_MIT_POS_SPD:
            parameters_valid = isfinite(command->position) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max;
            break;
        case DAMIAO_CTRL_POS:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max &&
                               command->speed >= limit->v_min &&
                               command->speed <= limit->v_max;
            break;
        case DAMIAO_CTRL_SPD:
            parameters_valid = isfinite(command->speed) &&
                               command->speed >= limit->v_min &&
                               command->speed <= limit->v_max;
            break;
        case DAMIAO_CTRL_PSI:
            parameters_valid = isfinite(command->position) &&
                               isfinite(command->speed) &&
                               isfinite(command->current) &&
                               command->position >= limit->p_min &&
                               command->position <= limit->p_max &&
                               command->speed >= 0.0f &&
                               command->speed <= limit->v_max &&
                               command->current >= 0.0f &&
                               command->current <= 6.5535f;
            break;
        default:
            break;
    }

    if (!parameters_valid) {
        DamiaoComm_SetEnabled(0U);
        damiao_motor.ctrl_mode = DAMIAO_CTRL_STOP;
        return;
    }

    damiao_motor.Pos = command->position;
    damiao_motor.W = command->speed;
    damiao_motor.T = command->torque;
    damiao_motor.K_P = command->kp;
    damiao_motor.K_W = command->kd;
    damiao_motor.Current = command->current;
    damiao_motor.ctrl_mode = command->control_mode;
    DamiaoComm_SetEnabled(1U);
}

void DamiaoComm_ProcessAndTransmit(void) {
    FDCAN_ErrorCode service_status = FDCAN_Service(&can1);

    if (service_status != FDCAN_ERR_NONE) {
        damiao_fdcan_start_error = service_status;
        return;
    }
    damiao_fdcan_start_error = FDCAN_ERR_NONE;

    DamiaoMotor_Control_Update(&damiao_motor);
    if (damiao_enabled_request == 0U) {
        return;
    }

    if (damiao_motor.flag.connected == 0U) {
        if (damiao_enable_retry_count == 0U) {
            DamiaoMotor_Enable(&damiao_motor);
            damiao_enable_retry_count = DAMIAO_COMM_ENABLE_RETRY_PERIOD;
        } else {
            damiao_enable_retry_count--;
        }

        /*
         * 达妙电机按控制帧返回反馈。尚未取得首帧反馈时也必须持续发送
         * MIT 控制帧，否则会形成“等待反馈后才发送、等待发送后才反馈”的死锁。
         */
        DamiaoMotor_Transmit(&damiao_motor);
        return;
    }

    if (damiao_motor.connected != DAMIAO_STATE_ENABLED) {
        if (damiao_enable_retry_count == 0U) {
            DamiaoMotor_Enable(&damiao_motor);
            damiao_enable_retry_count = DAMIAO_COMM_ENABLE_RETRY_PERIOD;
        } else {
            damiao_enable_retry_count--;
        }
        return;
    }

    damiao_enable_retry_count = 0U;
    DamiaoMotor_Transmit(&damiao_motor);
}
