/**
 * @file    DM_Motor.c
 * @brief   达妙伺服电机驱动库实现
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-06
 */
#include "DM_Motor.h"
#include "FDCAN_Basic.h"
#include "fdcan.h"
#include <string.h>

#define DAMIAO_MODE_BASE_MIT 0x000U
#define DAMIAO_MODE_BASE_POS 0x100U
#define DAMIAO_MODE_BASE_SPD 0x200U
#define DAMIAO_MODE_BASE_PSI 0x300U

#define DAMIAO_CMD_MOTOR_ENABLE 0xFCU
#define DAMIAO_CMD_MOTOR_DISABLE 0xFDU
#define DAMIAO_CMD_SAVE_ZERO 0xFEU
#define DAMIAO_CMD_CLEAR_ERROR 0xFBU

#define DAMIAO_CTRL_MODE_MASK(mode) ((uint16_t)(1UL << (uint32_t)(mode)))
#define DAMIAO_CTRL_MODE_MASK_ALL                                                \
    (DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_STOP) |                                  \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT) |                                   \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT_POS) |                               \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT_POS_SPD) |                           \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_POS) |                                   \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_SPD) |                                   \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_PSI))
#define DAMIAO_CTRL_MODE_MASK_G6220                                             \
    (DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_STOP) |                                  \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT) |                                   \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT_POS) |                               \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_MIT_POS_SPD) |                           \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_POS) |                                   \
     DAMIAO_CTRL_MODE_MASK(DAMIAO_CTRL_SPD))

static const DamiaoMotorModelParameters damiao_model_parameters[] = {
    {
        .type = DAMIAO_MOTOR_TYPE_DM4310,
        .limit = {
            DM4310_P_MIN, DM4310_P_MAX, DM4310_V_MIN, DM4310_V_MAX,
            DM4310_T_MIN, DM4310_T_MAX, DM4310_KP_MIN, DM4310_KP_MAX,
            DM4310_KD_MIN, DM4310_KD_MAX,
        },
        .can_bitrate_bps = DAMIAO_CAN_BITRATE_BPS,
        .supported_control_modes = DAMIAO_CTRL_MODE_MASK_ALL,
    },
    {
        .type = DAMIAO_MOTOR_TYPE_DM_J4340_2EC,
        .limit = {
            DM_J4340_2EC_P_MIN, DM_J4340_2EC_P_MAX,
            DM_J4340_2EC_V_MIN, DM_J4340_2EC_V_MAX,
            DM_J4340_2EC_T_MIN, DM_J4340_2EC_T_MAX,
            DM_J4340_2EC_KP_MIN, DM_J4340_2EC_KP_MAX,
            DM_J4340_2EC_KD_MIN, DM_J4340_2EC_KD_MAX,
        },
        .can_bitrate_bps = DAMIAO_CAN_BITRATE_BPS,
        .supported_control_modes = DAMIAO_CTRL_MODE_MASK_ALL,
    },
    {
        .type = DAMIAO_MOTOR_TYPE_DM_G6220,
        .limit = {
            DM_G6220_P_MIN, DM_G6220_P_MAX, DM_G6220_V_MIN, DM_G6220_V_MAX,
            DM_G6220_T_MIN, DM_G6220_T_MAX, DM_G6220_KP_MIN, DM_G6220_KP_MAX,
            DM_G6220_KD_MIN, DM_G6220_KD_MAX,
        },
        .rated_voltage_v = DM_G6220_RATED_VOLTAGE_V,
        .rated_current_a = DM_G6220_RATED_CURRENT_A,
        .peak_current_a = DM_G6220_PEAK_CURRENT_A,
        .rated_torque_nm = DM_G6220_RATED_TORQUE_NM,
        .peak_torque_nm = DM_G6220_PEAK_TORQUE_NM,
        .rated_speed_rpm = DM_G6220_RATED_SPEED_RPM,
        .max_no_load_speed_rpm = DM_G6220_MAX_NO_LOAD_SPEED_RPM,
        .reduction_ratio = DM_G6220_REDUCTION_RATIO,
        .phase_inductance_uh = DM_G6220_PHASE_INDUCTANCE_UH,
        .phase_resistance_ohm = DM_G6220_PHASE_RESISTANCE_OHM,
        .can_bitrate_bps = DAMIAO_CAN_BITRATE_BPS,
        .tuning_uart_baudrate = DM_G6220_TUNING_UART_BAUDRATE,
        .supported_control_modes = DAMIAO_CTRL_MODE_MASK_G6220,
        .pole_pairs = DM_G6220_POLE_PAIRS,
    },
};

// 定义内部使用的宏函数
#define LIMIT_MIN_MAX(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#define FLOAT_TO_UINT(x, x_min, x_max, bits) \
    ((int)(((x) - (x_min)) * ((float)((1 << (bits)) - 1)) / ((x_max) - (x_min))))
#define UINT_TO_FLOAT(x_int, x_min, x_max, bits) \
    (((float)(x_int)) * ((x_max) - (x_min)) / ((float)((1 << (bits)) - 1)) + (x_min))
static void DamiaoMotor_UpdateConnection(DamiaoMotor* motor);
static uint16_t DamiaoMotor_GetModeId(DamiaoMotorControlMode mode);
static void DamiaoMotor_SendFrame(DamiaoMotor* motor, uint32_t id, const uint8_t* data, uint8_t len);
static void DamiaoMotor_SendCommand(DamiaoMotor* motor, uint8_t command);
static uint8_t DamiaoMotor_PackMIT(DamiaoMotor* motor, uint8_t* buffer);
static uint8_t DamiaoMotor_PackPos(DamiaoMotor* motor, uint8_t* buffer);
static uint8_t DamiaoMotor_PackSpd(DamiaoMotor* motor, uint8_t* buffer);
static uint8_t DamiaoMotor_PackPSI(DamiaoMotor* motor, uint8_t* buffer);
static uint8_t DamiaoMotor_PackZeroMIT(DamiaoMotor* motor, uint8_t* buffer);

const DamiaoMotorModelParameters* DamiaoMotor_GetModelParameters(DamiaoMotorType type) {
    size_t index;

    for (index = 0U; index < (sizeof(damiao_model_parameters) /
                              sizeof(damiao_model_parameters[0]));
         index++) {
        if (damiao_model_parameters[index].type == type)
            return &damiao_model_parameters[index];
    }
    return NULL;
}

uint8_t DamiaoMotor_IsTypeSupported(DamiaoMotorType type) {
    return (DamiaoMotor_GetModelParameters(type) != NULL) ? 1U : 0U;
}

uint8_t DamiaoMotor_IsControlModeSupported(const DamiaoMotor* motor,
                                           DamiaoMotorControlMode mode) {
    if (motor == NULL || motor->model == NULL || mode > DAMIAO_CTRL_PSI)
        return 0U;

    return ((motor->model->supported_control_modes &
             DAMIAO_CTRL_MODE_MASK(mode)) != 0U) ? 1U : 0U;
}

// 绑定给定的配置到电机句柄并将其初始化
void DamiaoMotor_Init(DamiaoMotor* motor, DamiaoMotorType type, uint8_t can_bus, uint16_t id) {
    const DamiaoMotorModelParameters* model;

    if (motor == NULL)
        return;

    model = DamiaoMotor_GetModelParameters(type);
    memset(motor, 0, sizeof(DamiaoMotor));
    motor->can_bus = can_bus;
    motor->id = id;
    motor->mst_id = DAMIAO_MASTER_ID;
    motor->type = type;
    motor->model = model;
    motor->ctrl_mode = DAMIAO_CTRL_STOP;  // 默认初始化为STOP以防乱跑，但发送心跳帧使能
    if (model != NULL)
        motor->limit = model->limit;
}

void DamiaoMotor_Control_Update(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    if (DamiaoMotor_IsControlModeSupported(motor, motor->ctrl_mode) == 0U)
        return;

    DamiaoMotor_UpdateConnection(motor);

    switch (motor->ctrl_mode) {
        case DAMIAO_CTRL_MIT_POS:
            motor->W = Pid_Regulate_Auto(motor->Pos, motor->rx_Pos, &motor->PID.PosPID);

            motor->W = LIMIT_MIN_MAX(motor->W, motor->limit.v_min, motor->limit.v_max);
            motor->T = 0.0f;
            motor->flag.safe_switch = 1;
            break;

        case DAMIAO_CTRL_MIT_POS_SPD:
            motor->W = Pid_Regulate_Auto(motor->Pos, motor->rx_Pos, &motor->PID.PosPID);

            motor->T = Pid_Regulate_Auto(motor->W, motor->rx_W, &motor->PID.SpeedPID);

            motor->T = LIMIT_MIN_MAX(motor->T, motor->limit.t_min, motor->limit.t_max);
            motor->flag.safe_switch = 1;
            break;

        case DAMIAO_CTRL_MIT:
            if (motor->flag.safe_switch) {
                motor->Pos = motor->rx_Pos;  // 切回MIT模式时，默认以当前位置为目标位置
                motor->W = 0.0f;
                motor->T = 0.0f;
                motor->flag.safe_switch = 0;
            }
            break;

        case DAMIAO_CTRL_STOP:
            motor->W = 0.0f;
            motor->T = 0.0f;
            motor->Pos = motor->rx_Pos;
            break;

        default:
            break;
    }
}

// 使能电机，解除锁定并将电机力矩或闭环接管上线
void DamiaoMotor_Enable(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    DamiaoMotor_SendCommand(motor, DAMIAO_CMD_MOTOR_ENABLE);
}

// 外部主动使电机停止输出（类似急停或者怠速失能不工作），通常也用于重置其缓冲指令
void DamiaoMotor_Disable(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    DamiaoMotor_SendCommand(motor, DAMIAO_CMD_MOTOR_DISABLE);
    DamiaoMotor_ClearCommand(motor);
}

// 清除电机报警断电报错
void DamiaoMotor_ClearError(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    DamiaoMotor_SendCommand(motor, DAMIAO_CMD_CLEAR_ERROR);
}

// 为电机保存当前的机械初始姿态角度作为零点。
void DamiaoMotor_SaveZeroPosition(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    DamiaoMotor_SendCommand(motor, DAMIAO_CMD_SAVE_ZERO);
}

// 内部调用，清空用户设定的预期指令（常用于断开使能或触发保护之后清空缓冲）
void DamiaoMotor_ClearCommand(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    motor->T = 0.0f;
    motor->W = 0.0f;
    motor->Pos = 0.0f;
    motor->K_P = 0.0f;
    motor->K_W = 0.0f;
    motor->Current = 0.0f;
}

// 调用此函数主动发包——针对当前选择的控制模式构造对应的CAN包报文下发进消息队列
void DamiaoMotor_Transmit(DamiaoMotor* motor) {
    uint16_t can_id;
    uint8_t data[8] = {0};
    uint8_t len;

    if (motor == NULL)
        return;

    if (DamiaoMotor_IsControlModeSupported(motor, motor->ctrl_mode) == 0U)
        return;

    /*
     * 是否允许发送由应用层管理。这里不能以反馈在线状态作为发送前提：
     * 达妙电机需要先收到控制帧才会返回反馈，若在未收到首帧反馈时直接
     * return，会导致控制帧永远无法发出。
     */

    switch (motor->ctrl_mode) {
        case DAMIAO_CTRL_POS:
            len = DamiaoMotor_PackPos(motor, data);
            break;
        case DAMIAO_CTRL_SPD:
            len = DamiaoMotor_PackSpd(motor, data);
            break;
        case DAMIAO_CTRL_PSI:
            len = DamiaoMotor_PackPSI(motor, data);
            break;
        case DAMIAO_CTRL_STOP:
            len = DamiaoMotor_PackZeroMIT(motor, data);
            break;
        case DAMIAO_CTRL_MIT_POS:
        case DAMIAO_CTRL_MIT_POS_SPD:
        case DAMIAO_CTRL_MIT:
        default:
            len = DamiaoMotor_PackMIT(motor, data);
            break;
    }

    if (len == 0)
        return;

    can_id = motor->id + DamiaoMotor_GetModeId(motor->ctrl_mode);
    DamiaoMotor_SendFrame(motor, can_id, data, len);
}

// 解析从CAN中断中收取的8个字节数据报文（它反映了电机的当前报错码状态、位置、速度、转矩及温度）。
void DamiaoMotor_UnpackFeedback(DamiaoMotor* motor, const uint8_t* data, uint8_t len) {
    int p_int, v_int, t_int;

    if (motor == NULL || data == NULL || len < 8)
        return;

    motor->connected = (DamiaoMotorState)((data[0] >> 4) & 0x0F);
    p_int = ((int)data[1] << 8) | data[2];
    v_int = ((int)data[3] << 4) | (data[4] >> 4);
    t_int = (((int)data[4] & 0x0F) << 8) | data[5];
    {
        float pos_cycle = motor->limit.p_max - motor->limit.p_min;
        float pos_half_cycle = pos_cycle * 0.5f;
        float pos_raw = UINT_TO_FLOAT(p_int, motor->limit.p_min, motor->limit.p_max, 16);
        if (motor->rx_count == 0U) {
            motor->rx_Pos = pos_raw;
        } else {
            float delta = pos_raw - motor->Data.PosRaw;
            if (delta > pos_half_cycle) {
                delta -= pos_cycle;
            } else if (delta < -pos_half_cycle) {
                delta += pos_cycle;
            }

            motor->rx_Pos += delta;
        }
        motor->Data.PosRaw = pos_raw;
    }
    motor->rx_W = UINT_TO_FLOAT(v_int, motor->limit.v_min, motor->limit.v_max, 12);
    motor->rx_T = UINT_TO_FLOAT(t_int, motor->limit.t_min, motor->limit.t_max, 12);
    motor->rx_Tmos = (float)data[6];
    motor->rx_Tcoil = (float)data[7];
    motor->flag.feedback_ready = 1;
    motor->rx_count++;
    motor->flag.lost_count = 0U;
    motor->flag.connected = 1U;
}

// 通用数据帧处理入口，由CAN的RX回调调用检测，过滤掉来自无关CAN ID的心跳帧和杂波。
void Process_Damiao_Motor_Frame(FDCANFrame* frame, DamiaoMotor* motor) {
    if (frame == NULL || motor == NULL)
        return;

    if ((frame->Id.StdID & 0x7FFU) != motor->mst_id)
        return;

    if (frame->Length < 8)
        return;

    if ((frame->Data.uchars[0] & 0x0F) != motor->id)
        return;

    DamiaoMotor_UnpackFeedback(motor, frame->Data.uchars, frame->Length);
}

// 通用 FDCAN 分发器适配入口；context 指向对应的 DamiaoMotor 实例。
uint8_t DamiaoMotor_RxHandler(const FDCANFrame* frame, void* context) {
    DamiaoMotor* motor = (DamiaoMotor*)context;

    if (frame == NULL || motor == NULL || frame->canbus_id != motor->can_bus || frame->IDE != 0U ||
        frame->isRemote != 0 || frame->Length < 8U || (frame->Id.StdID & 0x7FFU) != motor->mst_id ||
        (frame->Data.uchars[0] & 0x0FU) != motor->id) {
        return 0U;
    }

    Process_Damiao_Motor_Frame((FDCANFrame*)frame, motor);
    return 1U;
}

/* ================== 内部函数实现 ================== */

// 根据反馈计数判断通讯是否仍在线，不依赖系统时间戳。
static void DamiaoMotor_UpdateConnection(DamiaoMotor* motor) {
    if (motor == NULL)
        return;

    if (motor->rx_count == 0U) {
        motor->flag.connected = 0U;
        return;
    }

    if (motor->rx_count != motor->flag.rx_count_last) {
        motor->flag.rx_count_last = motor->rx_count;
        motor->flag.lost_count = 0U;
        motor->flag.connected = 1U;
        return;
    }

    if (motor->flag.lost_count < UINT16_MAX)
        motor->flag.lost_count++;

    if (motor->flag.lost_count >= DAMIAO_FEEDBACK_LOST_LIMIT)
        motor->flag.connected = 0U;
}

// 根据控制模式获取对应的基础CAN ID（由于同一电机不同模式会需要不同的指令帧ID组）
static uint16_t DamiaoMotor_GetModeId(DamiaoMotorControlMode mode) {
    switch (mode) {
        case DAMIAO_CTRL_POS:
            return DAMIAO_MODE_BASE_POS;
        case DAMIAO_CTRL_SPD:
            return DAMIAO_MODE_BASE_SPD;
        case DAMIAO_CTRL_PSI:
            return DAMIAO_MODE_BASE_PSI;
        case DAMIAO_CTRL_MIT:
        case DAMIAO_CTRL_STOP:
        default:
            return DAMIAO_MODE_BASE_MIT;
    }
}

// 底层CAN发送函数，根据目标CAN总线通道封装FDCAN报文并放入操作系统的发送队列。
static void DamiaoMotor_SendFrame(DamiaoMotor* motor, uint32_t id, const uint8_t* data, uint8_t len) {
#if defined(USE_CAN_1) || defined(USE_CAN_2)
    if (len > 8)
        len = 8;

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = (id > 0x7FFU) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    TxHeader.Identifier = id;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_LengthToDataLength(len);

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (FDCAN_Transmit_Message(motor->can_bus, &TxHeader, (uint8_t*)data) == HAL_OK) {
        motor->tx_count++;
    }
#endif
}

// 发送特定的特殊指令码（如开启、关闭、清错、置零），它具有固定 8 字节FF的前缀格式
static void DamiaoMotor_SendCommand(DamiaoMotor* motor, uint8_t command) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
    uint16_t can_id = motor->id + DamiaoMotor_GetModeId(motor->ctrl_mode);

    DamiaoMotor_SendFrame(motor, can_id, data, 8);
}

// 打包MIT（多控制源：位置、速度、扭矩、Kp、Kd共同控制）模式下的指令数据阵列
static uint8_t DamiaoMotor_PackMIT(DamiaoMotor* motor, uint8_t* buffer) {
    uint16_t pos_tmp;
    uint16_t vel_tmp;
    uint16_t kp_tmp;
    uint16_t kd_tmp;
    uint16_t tor_tmp;

    float pos = LIMIT_MIN_MAX(motor->Pos, motor->limit.p_min, motor->limit.p_max);
    float vel = LIMIT_MIN_MAX(motor->W, motor->limit.v_min, motor->limit.v_max);
    float tor = LIMIT_MIN_MAX(motor->T, motor->limit.t_min, motor->limit.t_max);
    float kp = LIMIT_MIN_MAX(motor->K_P, motor->limit.kp_min, motor->limit.kp_max);
    float kd = LIMIT_MIN_MAX(motor->K_W, motor->limit.kd_min, motor->limit.kd_max);

    switch (motor->ctrl_mode) {
        case DAMIAO_CTRL_MIT_POS:
            kp = 0.0f;
            break;

        case DAMIAO_CTRL_MIT_POS_SPD:
            kp = 0.0f;
            kd = 0.0f;
            break;

        default:
            break;
    }

    pos_tmp = (uint16_t)FLOAT_TO_UINT(pos, motor->limit.p_min, motor->limit.p_max, 16);
    vel_tmp = (uint16_t)FLOAT_TO_UINT(vel, motor->limit.v_min, motor->limit.v_max, 12);
    kp_tmp = (uint16_t)FLOAT_TO_UINT(kp, motor->limit.kp_min, motor->limit.kp_max, 12);
    kd_tmp = (uint16_t)FLOAT_TO_UINT(kd, motor->limit.kd_min, motor->limit.kd_max, 12);
    tor_tmp = (uint16_t)FLOAT_TO_UINT(tor, motor->limit.t_min, motor->limit.t_max, 12);

    buffer[0] = (uint8_t)(pos_tmp >> 8);
    buffer[1] = (uint8_t)(pos_tmp);
    buffer[2] = (uint8_t)(vel_tmp >> 4);
    buffer[3] = (uint8_t)(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
    buffer[4] = (uint8_t)(kp_tmp);
    buffer[5] = (uint8_t)(kd_tmp >> 4);
    buffer[6] = (uint8_t)(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
    buffer[7] = (uint8_t)(tor_tmp);
    return 8;
}

// 打包位置(POS)环特化模式的下发帧（仅发目标位置及期望速度）。
static uint8_t DamiaoMotor_PackPos(DamiaoMotor* motor, uint8_t* buffer) {
    memcpy(&buffer[0], &motor->Pos, sizeof(float));
    memcpy(&buffer[4], &motor->W, sizeof(float));
    return 8;
}

// 打包速度(SPD)环特化模式的下发帧（仅发期望速度，占4字节）。
static uint8_t DamiaoMotor_PackSpd(DamiaoMotor* motor, uint8_t* buffer) {
    memcpy(&buffer[0], &motor->W, sizeof(float));
    return 4;
}

// 打包位置速度电流(PSI)环伺服模式：发位置、速度、最大电流限制）。
static uint8_t DamiaoMotor_PackPSI(DamiaoMotor* motor, uint8_t* buffer) {
    uint16_t vel_u16 = (uint16_t)(motor->W * 100.0f);
    uint16_t cur_u16 = (uint16_t)(motor->Current * 10000.0f);

    memcpy(&buffer[0], &motor->Pos, sizeof(float));
    memcpy(&buffer[4], &vel_u16, sizeof(uint16_t));
    memcpy(&buffer[6], &cur_u16, sizeof(uint16_t));
    return 8;
}

// 打包零力矩的MIT指令用于在停止状态下保持心跳反馈
static uint8_t DamiaoMotor_PackZeroMIT(DamiaoMotor* motor, uint8_t* buffer) {
    float saved_t = motor->T;
    float saved_w = motor->W;
    float saved_pos = motor->Pos;
    float saved_kp = motor->K_P;
    float saved_kw = motor->K_W;
    float saved_current = motor->Current;

    motor->T = 0.0f;
    motor->W = 0.0f;
    motor->Pos = 0.0f;
    motor->K_P = 0.0f;
    motor->K_W = 0.0f;
    motor->Current = 0.0f;
    uint8_t len = DamiaoMotor_PackMIT(motor, buffer);
    motor->T = saved_t;
    motor->W = saved_w;
    motor->Pos = saved_pos;
    motor->K_P = saved_kp;
    motor->K_W = saved_kw;
    motor->Current = saved_current;
    return len;
}
