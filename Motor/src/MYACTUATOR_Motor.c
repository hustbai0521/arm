#include "MYACTUATOR_Motor.h"

#include <limits.h>
#include <string.h>

#define MYACTUATOR_RAD_TO_DEG 57.29577951308232f
#define MYACTUATOR_DEG_TO_RAD 0.017453292519943295f

#define LIMIT_VALUE(x, lo, hi) (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))

static uint16_t MYACTUATOR_FloatToUint(float value, float min_value, float max_value, uint8_t bits) {
    float clamped = LIMIT_VALUE(value, min_value, max_value);
    uint32_t full_scale = (1UL << bits) - 1UL;
    return (uint16_t)((clamped - min_value) * (float)full_scale / (max_value - min_value));
}

static float MYACTUATOR_UintToFloat(uint16_t value, float min_value, float max_value, uint8_t bits) {
    uint32_t full_scale = (1UL << bits) - 1UL;
    return (float)value * (max_value - min_value) / (float)full_scale + min_value;
}

static int16_t MYACTUATOR_GetI16LE(const uint8_t* data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int32_t MYACTUATOR_GetI32LE(const uint8_t* data) {
    uint32_t value = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

static void MYACTUATOR_PutI16LE(uint8_t* data, int16_t value) {
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
}

static void MYACTUATOR_PutI32LE(uint8_t* data, int32_t value) {
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8);
    data[2] = (uint8_t)(raw >> 16);
    data[3] = (uint8_t)(raw >> 24);
}

static void MYACTUATOR_Send(MYACTUATOR_Motor* motor, uint32_t identifier, const uint8_t data[8]) {
#if defined(USE_CAN_1) || defined(USE_CAN_2)
    FDCAN_TxHeaderTypeDef header;

    if (motor == NULL || data == NULL) {
        return;
    }

    memset(&header, 0, sizeof(header));
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (FDCAN_Transmit_Message(motor->can_bus, &header, (uint8_t*)data) == HAL_OK) {
        motor->tx_count++;
    }
#else
    (void)motor;
    (void)identifier;
    (void)data;
#endif
}

static void MYACTUATOR_SendSimpleCommand(MYACTUATOR_Motor* motor, uint8_t command) {
    uint8_t data[8] = {0};
    if (motor == NULL) {
        return;
    }
    data[0] = command;
    MYACTUATOR_Send(motor, MYACTUATOR_COMMAND_ID_BASE + motor->id, data);
}

static void MYACTUATOR_SendTorque(MYACTUATOR_Motor* motor) {
    uint8_t data[8] = {0};
    float current;
    int16_t current_raw;

    data[0] = MYACTUATOR_CMD_TORQUE;
    current = motor->T / MYACTUATOR_X4_36_TORQUE_CONSTANT;
    current_raw = (int16_t)LIMIT_VALUE(current * 100.0f, (float)INT16_MIN, (float)INT16_MAX);
    MYACTUATOR_PutI16LE(&data[4], current_raw);
    MYACTUATOR_Send(motor, MYACTUATOR_COMMAND_ID_BASE + motor->id, data);
}

static void MYACTUATOR_SendSpeed(MYACTUATOR_Motor* motor) {
    uint8_t data[8] = {0};
    float speed_dps;
    int32_t speed_raw;

    data[0] = MYACTUATOR_CMD_SPEED;
    speed_dps = motor->W * MYACTUATOR_RAD_TO_DEG;
    speed_raw = (int32_t)(speed_dps * 100.0f);
    MYACTUATOR_PutI32LE(&data[4], speed_raw);
    MYACTUATOR_Send(motor, MYACTUATOR_COMMAND_ID_BASE + motor->id, data);
}

static void MYACTUATOR_SendPosition(MYACTUATOR_Motor* motor) {
    uint8_t data[8] = {0};
    float position_deg;
    float speed_dps;
    uint16_t speed_raw;
    int32_t position_raw;

    data[0] = MYACTUATOR_CMD_ABSOLUTE_POSITION;
    speed_dps = LIMIT_VALUE(motor->max_speed * MYACTUATOR_RAD_TO_DEG, 0.0f, 65535.0f);
    speed_raw = (uint16_t)speed_dps;
    data[2] = (uint8_t)speed_raw;
    data[3] = (uint8_t)(speed_raw >> 8);
    position_deg = motor->Pos * MYACTUATOR_RAD_TO_DEG;
    position_raw = (int32_t)(position_deg * 100.0f);
    MYACTUATOR_PutI32LE(&data[4], position_raw);
    MYACTUATOR_Send(motor, MYACTUATOR_COMMAND_ID_BASE + motor->id, data);
}

static void MYACTUATOR_SendMotion(MYACTUATOR_Motor* motor) {
    uint8_t data[8];
    uint16_t p;
    uint16_t v;
    uint16_t kp;
    uint16_t kd;
    uint16_t t;

    p = MYACTUATOR_FloatToUint(motor->Pos, MYACTUATOR_MOTION_P_MIN, MYACTUATOR_MOTION_P_MAX, 16U);
    v = MYACTUATOR_FloatToUint(motor->W, MYACTUATOR_MOTION_V_MIN, MYACTUATOR_MOTION_V_MAX, 12U);
    kp = MYACTUATOR_FloatToUint(motor->K_P, MYACTUATOR_MOTION_KP_MIN, MYACTUATOR_MOTION_KP_MAX, 12U);
    kd = MYACTUATOR_FloatToUint(motor->K_W, MYACTUATOR_MOTION_KD_MIN, MYACTUATOR_MOTION_KD_MAX, 12U);
    t = MYACTUATOR_FloatToUint(motor->T, MYACTUATOR_MOTION_T_MIN, MYACTUATOR_MOTION_T_MAX, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp >> 8));
    data[4] = (uint8_t)kp;
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    MYACTUATOR_Send(motor, MYACTUATOR_MOTION_COMMAND_ID_BASE + motor->id, data);
}

static void MYACTUATOR_UpdateFeedbackReady(MYACTUATOR_Motor* motor, uint8_t command) {
    motor->last_command = command;
    motor->rx_count++;
    motor->flag.feedback_ready = 1U;
    motor->flag.connected = 1U;
    motor->flag.lost_count = 0U;
}

static void MYACTUATOR_ParseStatus2(MYACTUATOR_Motor* motor, const uint8_t* data) {
    int16_t current_raw = MYACTUATOR_GetI16LE(&data[2]);
    int16_t speed_raw = MYACTUATOR_GetI16LE(&data[4]);
    int16_t angle_raw = MYACTUATOR_GetI16LE(&data[6]);

    motor->rx_Temp = (float)(int8_t)data[1];
    motor->rx_Current = (float)current_raw * 0.01f;
    motor->rx_T = motor->rx_Current * MYACTUATOR_X4_36_TORQUE_CONSTANT;
    motor->rx_W = (float)speed_raw * MYACTUATOR_DEG_TO_RAD;
    motor->rx_Pos = (float)angle_raw * MYACTUATOR_DEG_TO_RAD;
}

static void MYACTUATOR_ParseMotion(MYACTUATOR_Motor* motor, const uint8_t* data) {
    uint16_t p = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t t = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

    motor->rx_Pos = MYACTUATOR_UintToFloat(p, MYACTUATOR_MOTION_P_MIN, MYACTUATOR_MOTION_P_MAX, 16U);
    motor->rx_W = MYACTUATOR_UintToFloat(v, MYACTUATOR_MOTION_V_MIN, MYACTUATOR_MOTION_V_MAX, 12U);
    motor->rx_T = MYACTUATOR_UintToFloat(t, MYACTUATOR_MOTION_T_MIN, MYACTUATOR_MOTION_T_MAX, 12U);
    motor->rx_Current = motor->rx_T / MYACTUATOR_X4_36_TORQUE_CONSTANT;
}

void MYACTUATOR_Motor_Init(MYACTUATOR_Motor* motor, uint8_t can_bus, uint8_t id) {
    if (motor == NULL) {
        return;
    }
    memset(motor, 0, sizeof(*motor));
    motor->can_bus = can_bus;
    motor->id = id;
    motor->ctrl_mode = MYACTUATOR_CTRL_STOP;
}

void MYACTUATOR_Motor_ControlUpdate(MYACTUATOR_Motor* motor) {
    if (motor == NULL) {
        return;
    }
    if (motor->rx_count != motor->flag.last_rx_count) {
        motor->flag.last_rx_count = motor->rx_count;
        motor->flag.lost_count = 0U;
        motor->flag.connected = 1U;
    } else if (motor->rx_count != 0U) {
        if (motor->flag.lost_count < UINT16_MAX) {
            motor->flag.lost_count++;
        }
        if (motor->flag.lost_count >= MYACTUATOR_FEEDBACK_LOST_LIMIT) {
            motor->flag.connected = 0U;
        }
    }
}

void MYACTUATOR_Motor_Transmit(MYACTUATOR_Motor* motor) {
    if (motor == NULL || motor->id < MYACTUATOR_CAN_ID_MIN || motor->id > MYACTUATOR_CAN_ID_MAX) {
        return;
    }

    MYACTUATOR_Motor_ControlUpdate(motor);
    switch (motor->ctrl_mode) {
        case MYACTUATOR_CTRL_TORQUE:
            MYACTUATOR_SendTorque(motor);
            break;
        case MYACTUATOR_CTRL_SPEED:
            MYACTUATOR_SendSpeed(motor);
            break;
        case MYACTUATOR_CTRL_POSITION:
            MYACTUATOR_SendPosition(motor);
            break;
        case MYACTUATOR_CTRL_MOTION:
            MYACTUATOR_SendMotion(motor);
            break;
        case MYACTUATOR_CTRL_STOP:
        default:
            MYACTUATOR_Motor_Stop(motor);
            break;
    }
}

void MYACTUATOR_Motor_Shutdown(MYACTUATOR_Motor* motor) {
    MYACTUATOR_SendSimpleCommand(motor, MYACTUATOR_CMD_SHUTDOWN);
}

void MYACTUATOR_Motor_Stop(MYACTUATOR_Motor* motor) {
    MYACTUATOR_SendSimpleCommand(motor, MYACTUATOR_CMD_STOP);
}

void MYACTUATOR_Motor_ReadStatus(MYACTUATOR_Motor* motor) {
    MYACTUATOR_SendSimpleCommand(motor, MYACTUATOR_CMD_READ_STATUS_2);
}

void MYACTUATOR_Motor_ReadMultiTurnAngle(MYACTUATOR_Motor* motor) {
    MYACTUATOR_SendSimpleCommand(motor, MYACTUATOR_CMD_READ_MULTI_TURN_ANGLE);
}

uint8_t MYACTUATOR_Motor_ProcessFrame(const FDCANFrame* frame, MYACTUATOR_Motor* motor) {
    uint32_t identifier;
    uint8_t command;

    if (frame == NULL || motor == NULL || frame->canbus_id != motor->can_bus || frame->IDE != 0U ||
        frame->isRemote != 0 || frame->Length < 8U) {
        return 0U;
    }

    identifier = frame->Id.StdID & 0x7FFU;
    if (identifier == MYACTUATOR_MOTION_REPLY_ID_BASE + motor->id) {
        if (frame->Data.uchars[0] != motor->id) {
            return 0U;
        }
        MYACTUATOR_ParseMotion(motor, frame->Data.uchars);
        MYACTUATOR_UpdateFeedbackReady(motor, 0U);
        return 1U;
    }

    if (identifier != MYACTUATOR_REPLY_ID_BASE + motor->id) {
        return 0U;
    }

    command = frame->Data.uchars[0];
    switch (command) {
        case MYACTUATOR_CMD_TORQUE:
        case MYACTUATOR_CMD_SPEED:
        case MYACTUATOR_CMD_ABSOLUTE_POSITION:
        case MYACTUATOR_CMD_READ_STATUS_2:
            MYACTUATOR_ParseStatus2(motor, frame->Data.uchars);
            break;
        case MYACTUATOR_CMD_READ_MULTI_TURN_ANGLE: {
            int32_t angle_raw = MYACTUATOR_GetI32LE(&frame->Data.uchars[4]);
            motor->rx_Pos = (float)angle_raw * 0.01f * MYACTUATOR_DEG_TO_RAD;
            break;
        }
        case MYACTUATOR_CMD_READ_STATUS_1:
            motor->rx_Temp = (float)(int8_t)frame->Data.uchars[1];
            motor->rx_Voltage = (float)(uint16_t)MYACTUATOR_GetI16LE(&frame->Data.uchars[4]) * 0.1f;
            motor->rx_Error = (uint16_t)MYACTUATOR_GetI16LE(&frame->Data.uchars[6]);
            break;
        case MYACTUATOR_CMD_SHUTDOWN:
        case MYACTUATOR_CMD_STOP:
            break;
        default:
            return 0U;
    }

    MYACTUATOR_UpdateFeedbackReady(motor, command);
    return 1U;
}

uint8_t MYACTUATOR_Motor_RxHandler(const FDCANFrame* frame, void* context) {
    return MYACTUATOR_Motor_ProcessFrame(frame, (MYACTUATOR_Motor*)context);
}
