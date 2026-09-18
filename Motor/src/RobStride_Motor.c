/**
 * @file    RobStride_Motor.c
 * @brief   RobStride motor driver implementation
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-26
 */
#include "RobStride_Motor.h"
#include <string.h>

#define LIMIT_MIN_MAX(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#define FLOAT_TO_UINT(x, x_min, x_max, bits) \
    ((int)(((x) - (x_min)) * ((float)((1 << (bits)) - 1)) / ((x_max) - (x_min))))
#define UINT_TO_FLOAT(x_int, x_min, x_max, bits) \
    (((float)(x_int)) * ((x_max) - (x_min)) / ((float)((1 << (bits)) - 1)) + (x_min))
#define ROBSTRIDE_POS_CYCLE (RS_LIMIT_P_MAX - RS_LIMIT_P_MIN)
#define ROBSTRIDE_POS_HALF_CYCLE (ROBSTRIDE_POS_CYCLE * 0.5f)

/**
 * @brief  配置电机型号限制与默认 PID
 * @param  motor: 电机对象指针
 * @retval 无
 */
static void RobStrideMotor_ConfigType(RobStrideMotor* motor) {
    RobStrideMotorPID* pid = &motor->PID;
    RobStrideMotorLimit* limit = &motor->Data.limit;

    pid->PosPID.Kp = 5.0f;
    pid->PosPID.Ki = 0.0f;
    pid->PosPID.Kd = 0.0f;
    pid->PosPID.Integral = 0.0f;
    pid->PosPID.LimitOutput = 10.0f;
    pid->PosPID.LimitIntegral = 2.0f;
    pid->PosPID.PreError = 0.0f;

    pid->SpeedPID.Kp = 0.1f;
    pid->SpeedPID.Ki = 0.01f;
    pid->SpeedPID.Kd = 0.0f;
    pid->SpeedPID.Integral = 0.0f;
    pid->SpeedPID.LimitOutput = 5.0f;
    pid->SpeedPID.LimitIntegral = 1.0f;
    pid->SpeedPID.PreError = 0.0f;

    switch (motor->type) {
        case ROBSTRIDE_TYPE_RS00:
            limit->p_min = RS_LIMIT_P_MIN;
            limit->p_max = RS_LIMIT_P_MAX;
            limit->v_min = RS_LIMIT_V_MIN;
            limit->v_max = RS_LIMIT_V_MAX;
            limit->t_min = RS_LIMIT_T_MIN;
            limit->t_max = RS_LIMIT_T_MAX;
            limit->kp_min = RS_LIMIT_KP_MIN;
            limit->kp_max = RS_LIMIT_KP_MAX;
            limit->kd_min = RS_LIMIT_KD_MIN;
            limit->kd_max = RS_LIMIT_KD_MAX;
            break;

        case ROBSTRIDE_TYPE_RS05:
            limit->p_min = RS_LIMIT_P_MIN;
            limit->p_max = RS_LIMIT_P_MAX;
            limit->v_min = RS05_LIMIT_V_MIN;
            limit->v_max = RS05_LIMIT_V_MAX;
            limit->t_min = RS05_LIMIT_T_MIN;
            limit->t_max = RS05_LIMIT_T_MAX;
            limit->kp_min = RS_LIMIT_KP_MIN;
            limit->kp_max = RS_LIMIT_KP_MAX;
            limit->kd_min = RS_LIMIT_KD_MIN;
            limit->kd_max = RS_LIMIT_KD_MAX;
            break;

        case ROBSTRIDE_TYPE_RS01:
            /* O1 与其他 RobStride 型号共用协议，但 MIT 量化必须使用 O1 自己的范围。 */
            limit->p_min = RS_LIMIT_P_MIN;
            limit->p_max = RS_LIMIT_P_MAX;
            limit->v_min = RS01_LIMIT_V_MIN;
            limit->v_max = RS01_LIMIT_V_MAX;
            limit->t_min = RS01_LIMIT_T_MIN;
            limit->t_max = RS01_LIMIT_T_MAX;
            limit->kp_min = RS_LIMIT_KP_MIN;
            limit->kp_max = RS_LIMIT_KP_MAX;
            limit->kd_min = RS_LIMIT_KD_MIN;
            limit->kd_max = RS_LIMIT_KD_MAX;
            break;

        case ROBSTRIDE_TYPE_EL05:
            limit->p_min = RS_LIMIT_P_MIN;
            limit->p_max = RS_LIMIT_P_MAX;
            limit->v_min = EL05_LIMIT_V_MIN;
            limit->v_max = EL05_LIMIT_V_MAX;
            limit->t_min = EL05_LIMIT_T_MIN;
            limit->t_max = EL05_LIMIT_T_MAX;
            limit->kp_min = RS_LIMIT_KP_MIN;
            limit->kp_max = RS_LIMIT_KP_MAX;
            limit->kd_min = RS_LIMIT_KD_MIN;
            limit->kd_max = RS_LIMIT_KD_MAX;
            break;

        default:
            limit->p_min = RS_LIMIT_P_MIN;
            limit->p_max = RS_LIMIT_P_MAX;
            limit->v_min = RS_LIMIT_V_MIN;
            limit->v_max = RS_LIMIT_V_MAX;
            limit->t_min = RS_LIMIT_T_MIN;
            limit->t_max = RS_LIMIT_T_MAX;
            limit->kp_min = RS_LIMIT_KP_MIN;
            limit->kp_max = RS_LIMIT_KP_MAX;
            limit->kd_min = RS_LIMIT_KD_MIN;
            limit->kd_max = RS_LIMIT_KD_MAX;
            break;
    }
}

/**
 * @brief  更新电机在线状态
 * @param  motor: 电机对象指针
 * @retval 无
 */
static void RobStrideMotor_UpdateConn(RobStrideMotor* motor) {
    if (motor == NULL)
        return;

    if (motor->rx_count != motor->flag.last_rx_count) {
        motor->flag.last_rx_count = motor->rx_count;
        motor->flag.lost_count = 0U;
        motor->flag.connected = 1U;
    } else {
        if (motor->flag.lost_count < UINT16_MAX)
            motor->flag.lost_count++;

        if (motor->flag.lost_count >= ROBSTRIDE_FEEDBACK_LOST_LIMIT)
            motor->flag.connected = 0U;
    }
}

/**
 * @brief  发送一帧 RobStride CAN 数据
 * @param  bus: CAN 总线号 (1 ~ 2)
 * @param  id:  CAN 标识符
 * @param  data: 数据缓冲区
 * @param  len:  数据长度 (字节, 0 ~ 8)
 * @retval 无
 */
static bool RobStrideMotor_SendRaw(uint8_t bus, uint32_t id, const uint8_t* data, uint8_t len) {
#if defined(USE_CAN_1) || defined(USE_CAN_2)
    FDCAN_TxHeaderTypeDef header;

    if (data == NULL)
        return false;

    if (len > 8U)
        len = 8U;

    /* RobStride 私有协议始终使用 29 位扩展帧，不能按 ID 数值大小推断帧类型。 */
    header.IdType = FDCAN_EXTENDED_ID;
    header.Identifier = id & 0x1FFFFFFFU;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_LengthToDataLength(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    return (FDCAN_Transmit_Message(bus, &header, (uint8_t*)data) == HAL_OK);
#else
    (void)bus;
    (void)id;
    (void)data;
    (void)len;
    return false;
#endif
}

/**
 * @brief  通过电机对象发送一帧 RobStride 数据
 * @param  motor: 电机对象指针
 * @param  id:    CAN 标识符
 * @param  data:  数据缓冲区
 * @param  len:   数据长度 (字节, 0 ~ 8)
 * @retval 无
 */
static void RobStrideMotor_SendFrame(RobStrideMotor* motor, uint32_t id, const uint8_t* data, uint8_t len) {
    if (motor == NULL || data == NULL)
        return;

    RobStrideMotor_UpdateConn(motor);
    if (RobStrideMotor_SendRaw(motor->can_bus, id, data, len))
        motor->tx_count++;
}

/**
 * @brief  发送简单命令帧
 * @param  motor: 电机对象指针
 * @param  cmd:   命令号
 * @param  first: 首字节数据
 * @retval 无
 */
static void RobStrideMotor_SendCmd(RobStrideMotor* motor, uint8_t cmd, uint8_t first) {
    uint8_t data[8] = {0};
    uint32_t id;

    if (motor == NULL)
        return;

    data[0] = first;
    id = ((uint32_t)cmd << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)motor->mst_id << ROBSTRIDE_ID_OFFSET) | motor->id;
    RobStrideMotor_SendFrame(motor, id, data, 8);
}

/**
 * @brief  发送指定目标 ID 的读号帧
 * @param  motor: 电机对象指针
 * @param  scan_id: 扫描目标 ID (0 ~ 127)
 * @retval 无
 */
static void RobStrideMotor_SendScan(RobStrideMotor* motor, uint8_t scan_id) {
    uint8_t data[8] = {0};
    uint32_t id;

    if (motor == NULL)
        return;

    id = ((uint32_t)RS_CMD_GET_ID << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)motor->mst_id << ROBSTRIDE_ID_OFFSET) | scan_id;
    RobStrideMotor_SendFrame(motor, id, data, 8);
}

/**
 * @brief  发送改号帧
 * @param  bus: CAN 总线号 (1 ~ 2)
 * @param  old_id: 当前电机 ID (0 ~ 127)
 * @param  new_id: 新电机 ID (0 ~ 127)
 * @retval 无
 */
static void RobStrideMotor_SendSetID(uint8_t bus, uint8_t old_id, uint8_t new_id) {
    uint8_t data[8] = {0};
    uint32_t id;

    id = ((uint32_t)RS_CMD_SET_ID << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)new_id << 16) |
         ((uint32_t)ROBSTRIDE_MASTER_ID << ROBSTRIDE_ID_OFFSET) | old_id;
    (void)RobStrideMotor_SendRaw(bus, id, data, 8);
}

/**
 * @brief  解析扫描回复 ID
 * @param  frame: 接收帧指针
 * @retval 目标 ID (成功)
 * @retval 0xFFFF (不是扫描回复)
 */
static uint16_t RobStrideMotor_ParseScan(const FDCANFrame* frame) {
    uint32_t id;
    uint8_t cmd;
    uint16_t scan_id;

    if (frame == NULL)
        return ROBSTRIDE_SCAN_ERROR_ID;

    id = frame->Id.all;
    cmd = (id >> ROBSTRIDE_CMD_OFFSET) & 0x1FU;

    if (cmd != RS_CMD_GET_ID)
        return ROBSTRIDE_SCAN_ERROR_ID;

    if ((id & 0xFFU) != ROBSTRIDE_MASTER_ID)
        return ROBSTRIDE_SCAN_ERROR_ID;

    scan_id = (uint16_t)((id >> ROBSTRIDE_ID_OFFSET) & 0xFFFFU);
    if (scan_id > ROBSTRIDE_MAX_MOTOR_ID)
        return ROBSTRIDE_SCAN_ERROR_ID;

    return scan_id;
}

/**
 * @brief  打包 MIT 控制帧
 * @param  motor: 电机对象指针
 * @param  data:  输出数据缓冲区
 * @param  id:    输出标识符指针
 * @param  kp:    刚度系数
 * @param  kd:    阻尼系数
 * @retval 数据长度 (字节)
 */
static uint8_t RobStrideMotor_PackMIT(RobStrideMotor* motor, uint8_t* data, uint32_t* id, float kp, float kd) {
    uint16_t pos_u16;
    uint16_t spd_u16;
    uint16_t kp_u16;
    uint16_t kd_u16;
    uint16_t trq_u16;
    RobStrideMotorLimit* limit;
    float pos;
    float spd;
    float tor;

    limit = &motor->Data.limit;
    pos = LIMIT_MIN_MAX(motor->Pos, limit->p_min, limit->p_max);
    spd = LIMIT_MIN_MAX(motor->W, limit->v_min, limit->v_max);
    tor = LIMIT_MIN_MAX(motor->T, limit->t_min, limit->t_max);
    kp = LIMIT_MIN_MAX(kp, limit->kp_min, limit->kp_max);
    kd = LIMIT_MIN_MAX(kd, limit->kd_min, limit->kd_max);

    pos_u16 = (uint16_t)FLOAT_TO_UINT(pos, limit->p_min, limit->p_max, ROBSTRIDE_QUANT_BITS);
    spd_u16 = (uint16_t)FLOAT_TO_UINT(spd, limit->v_min, limit->v_max, ROBSTRIDE_QUANT_BITS);
    kp_u16 = (uint16_t)FLOAT_TO_UINT(kp, limit->kp_min, limit->kp_max, ROBSTRIDE_QUANT_BITS);
    kd_u16 = (uint16_t)FLOAT_TO_UINT(kd, limit->kd_min, limit->kd_max, ROBSTRIDE_QUANT_BITS);
    trq_u16 = (uint16_t)FLOAT_TO_UINT(tor, limit->t_min, limit->t_max, ROBSTRIDE_QUANT_BITS);

    *id = ((uint32_t)RS_CMD_CONTROL << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)trq_u16 << ROBSTRIDE_ID_OFFSET) | motor->id;
    data[0] = (uint8_t)(pos_u16 >> 8);
    data[1] = (uint8_t)(pos_u16);
    data[2] = (uint8_t)(spd_u16 >> 8);
    data[3] = (uint8_t)(spd_u16);
    data[4] = (uint8_t)(kp_u16 >> 8);
    data[5] = (uint8_t)(kp_u16);
    data[6] = (uint8_t)(kd_u16 >> 8);
    data[7] = (uint8_t)(kd_u16);
    return 8U;
}

/**
 * @brief  推进扫描索引
 * @param  motor: 电机对象指针
 * @retval 无
 */
static void RobStrideMotor_StepScan(RobStrideMotor* motor) {
    if (motor->Data.scan_idx >= ROBSTRIDE_MAX_MOTOR_ID)
        motor->Data.scan_idx = 0U;
    else
        motor->Data.scan_idx++;

    if (motor->Data.scan_idx == motor->Data.scan_start)
        motor->Data.scan_round++;
}

/**
 * @brief  检查扫描是否结束
 * @param  motor: 电机对象指针
 * @retval 无
 */
static void RobStrideMotor_CheckScanDone(RobStrideMotor* motor) {
    if (motor == NULL)
        return;

    if (motor->Data.scan_hit >= ROBSTRIDE_SCAN_MAX_HIT) {
        motor->Data.scan_ok = true;
        motor->ctrl_mode = ROBSTRIDE_CTRL_STOP;
        motor->Data.work_mode = ROBSTRIDE_WORK_RUN;
        return;
    }

    if (motor->Data.scan_round >= ROBSTRIDE_SCAN_MAX_ROUND) {
        motor->Data.scan_ok = false;
        motor->Data.scan_hit = 0U;
        motor->Data.scan_id = ROBSTRIDE_SCAN_ERROR_ID;
        motor->ctrl_mode = ROBSTRIDE_CTRL_STOP;
        motor->Data.work_mode = ROBSTRIDE_WORK_RUN;
    }
}

/**
 * @brief  处理扫描回复
 * @param  motor: 电机对象指针
 * @param  frame: 接收帧指针
 * @retval true (已处理为扫描回复)
 * @retval false (不是扫描回复)
 */
static bool RobStrideMotor_HandleScan(RobStrideMotor* motor, const FDCANFrame* frame) {
    uint16_t scan_id;

    if (motor == NULL || motor->Data.work_mode != ROBSTRIDE_WORK_SCAN)
        return false;

    scan_id = RobStrideMotor_ParseScan(frame);
    if (scan_id == ROBSTRIDE_SCAN_ERROR_ID)
        return false;

    if (motor->Data.scan_hit == 0U) {
        motor->Data.scan_id = scan_id;
        motor->Data.scan_hit = 1U;
        motor->Data.scan_ok = true;
        motor->id = (uint16_t)scan_id;
    } else if (motor->Data.scan_id == scan_id) {
        if (motor->Data.scan_hit < UINT8_MAX)
            motor->Data.scan_hit++;
        motor->id = (uint16_t)scan_id;
    }

    RobStrideMotor_CheckScanDone(motor);
    return true;
}

/**
 * @brief  初始化 RobStride 电机对象
 * @param  motor: 电机对象指针
 * @param  type:  电机型号
 * @param  can_bus: CAN 总线号 (1 ~ 2)
 * @param  id:    电机 ID (0 ~ 127)
 * @retval 无
 */
void RobStrideMotor_Init(RobStrideMotor* motor, RobStrideMotorType type, uint8_t can_bus, uint32_t id) {
    if (motor == NULL)
        return;

    memset(motor, 0, sizeof(RobStrideMotor));
    motor->id = (uint16_t)id;
    motor->mst_id = ROBSTRIDE_MASTER_ID;
    motor->can_bus = can_bus;
    motor->type = type;
    motor->ctrl_mode = ROBSTRIDE_CTRL_STOP;
    motor->flag.connected = 0U;
    motor->flag.lost_count = 0U;
    motor->flag.last_rx_count = 0U;
    motor->Data.rx_mode_state = ROBSTRIDE_STATE_RESET;
    motor->Data.rx_error_code = 0U;
    motor->Data.work_mode = ROBSTRIDE_WORK_RUN;
    motor->Data.scan_ok = false;
    motor->Data.scan_id = ROBSTRIDE_SCAN_ERROR_ID;
    motor->Data.scan_hit = 0U;
    motor->Data.scan_idx = (uint8_t)id;
    motor->Data.scan_start = (uint8_t)id;
    motor->Data.scan_round = 0U;

    RobStrideMotor_ConfigType(motor);
    /* 初始化只建立软件对象；使能时机由应用层统一管理。 */
}

/**
 * @brief  更新 RobStride 电机控制量
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Control_Update(RobStrideMotor* motor) {
    RobStrideMotorLimit* limit;

    if (motor == NULL)
        return;

    if (motor->Data.work_mode == ROBSTRIDE_WORK_SCAN) {
        motor->T = 0.0f;
        motor->W = 0.0f;
        return;
    }

    limit = &motor->Data.limit;

    switch (motor->ctrl_mode) {
        case ROBSTRIDE_CTRL_POS:
            motor->W = Pid_Regulate_Auto(motor->Pos, motor->rx_Pos, &motor->PID.PosPID);
            motor->W = LIMIT_MIN_MAX(motor->W, limit->v_min, limit->v_max);
            motor->T = 0.0f;
            break;

        case ROBSTRIDE_CTRL_SPD:
            motor->T = Pid_Regulate_Auto(motor->W, motor->rx_W, &motor->PID.SpeedPID);
            motor->T = LIMIT_MIN_MAX(motor->T, limit->t_min, limit->t_max);
            break;

        case ROBSTRIDE_CTRL_NATIVE_MIT:
            break;

        case ROBSTRIDE_CTRL_STOP:
            motor->T = 0.0f;
            motor->W = 0.0f;
            break;

        default:
            break;
    }
}

/**
 * @brief  发送 RobStride 电机控制或扫描帧
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Transmit(RobStrideMotor* motor) {
    uint8_t data[8] = {0};
    uint32_t id;
    uint8_t len;
    float kp;
    float kd;

    if (motor == NULL)
        return;

    if (motor->Data.work_mode == ROBSTRIDE_WORK_SCAN) {
        RobStrideMotor_SendScan(motor, motor->Data.scan_idx);
        RobStrideMotor_StepScan(motor);
        RobStrideMotor_CheckScanDone(motor);
        return;
    }

    if (motor->Data.rx_mode_state != ROBSTRIDE_STATE_RUN) {
        RobStrideMotor_Enable(motor);
        return;
    }

    kp = motor->K_P;
    kd = motor->K_W;

    switch (motor->ctrl_mode) {
        case ROBSTRIDE_CTRL_POS:
            kp = 0.0f;
            break;

        case ROBSTRIDE_CTRL_SPD:
            kp = 0.0f;
            kd = 0.0f;
            break;

        case ROBSTRIDE_CTRL_STOP:
            kp = 0.0f;
            kd = 0.0f;
            break;

        case ROBSTRIDE_CTRL_NATIVE_MIT:
        default:
            break;
    }

    len = RobStrideMotor_PackMIT(motor, data, &id, kp, kd);
    RobStrideMotor_SendFrame(motor, id, data, len);
}

/**
 * @brief  使能 RobStride 电机
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Enable(RobStrideMotor* motor) {
    RobStrideMotor_SendCmd(motor, RS_CMD_ENABLE, 0x00U);
}

/**
 * @brief  失能 RobStride 电机
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Disable(RobStrideMotor* motor) {
    RobStrideMotor_SendCmd(motor, RS_CMD_DISABLE, 0x00U);
}

/**
 * @brief  读取 RobStride 电机 ID
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_GetID(RobStrideMotor* motor) {
    uint8_t data[8] = {0};
    uint32_t id;

    if (motor == NULL)
        return;

    id = ((uint32_t)RS_CMD_GET_ID << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)motor->mst_id << ROBSTRIDE_ID_OFFSET) |
         motor->id;
    RobStrideMotor_SendFrame(motor, id, data, 8);
}

/**
 * @brief  修改 RobStride 电机 ID
 * @param  bus:    CAN 总线号 (1 ~ 2)
 * @param  old_id: 当前电机 ID (0 ~ 127)
 * @param  new_id: 新电机 ID (0 ~ 127)
 * @retval 无
 */
void RobStrideMotor_SetID(uint8_t bus, uint8_t old_id, uint8_t new_id) {
    if (old_id > ROBSTRIDE_MAX_MOTOR_ID || new_id > ROBSTRIDE_MAX_MOTOR_ID)
        return;

    RobStrideMotor_SendSetID(bus, old_id, new_id);
}

/**
 * @brief  设置当前位置为零点
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_SetZero(RobStrideMotor* motor) {
    RobStrideMotor_SendCmd(motor, RS_CMD_SET_ZERO, 0x01U);
}

/**
 * @brief  写入 RobStride RAM 参数
 * @param  motor: 电机对象指针
 * @param  index: 参数索引
 * @param  value: 参数值
 * @retval 无
 */
void RobStrideMotor_WriteParam(RobStrideMotor* motor, uint16_t index, float value) {
    uint8_t data[8] = {0};
    uint32_t id;

    if (motor == NULL)
        return;

    id = ((uint32_t)RS_CMD_WRITE_PARAM << ROBSTRIDE_CMD_OFFSET) | ((uint32_t)motor->mst_id << ROBSTRIDE_ID_OFFSET) |
         motor->id;
    memcpy(&data[0], &index, 2);
    memcpy(&data[4], &value, 4);
    RobStrideMotor_SendFrame(motor, id, data, 8);
}

/**
 * @brief  启动 RobStride ID 扫描
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_StartScan(RobStrideMotor* motor) {
    if (motor == NULL)
        return;

    motor->Data.work_mode = ROBSTRIDE_WORK_SCAN;
    motor->Data.scan_ok = false;
    motor->Data.scan_id = ROBSTRIDE_SCAN_ERROR_ID;
    motor->Data.scan_hit = 0U;
    motor->Data.scan_idx = (uint8_t)motor->id;
    motor->Data.scan_start = (uint8_t)motor->id;
    motor->Data.scan_round = 0U;
    motor->ctrl_mode = ROBSTRIDE_CTRL_STOP;
    motor->T = 0.0f;
    motor->W = 0.0f;
}

/**
 * @brief  停止 RobStride ID 扫描
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_StopScan(RobStrideMotor* motor) {
    if (motor == NULL)
        return;

    motor->Data.work_mode = ROBSTRIDE_WORK_RUN;
    motor->Data.scan_hit = 0U;
    motor->Data.scan_round = 0U;
    motor->Data.scan_idx = (uint8_t)motor->id;
    motor->Data.scan_start = (uint8_t)motor->id;
    motor->ctrl_mode = ROBSTRIDE_CTRL_STOP;
    motor->T = 0.0f;
    motor->W = 0.0f;
}

/**
 * @brief  解析 RobStride 电机接收帧
 * @param  frame: 接收帧指针
 * @param  motor: 电机对象指针
 * @retval 无
 */
void Process_RobStride_Motor_Frame(const FDCANFrame* frame, RobStrideMotor* motor) {
    uint32_t id;
    uint8_t cmd;
    uint8_t sender_id;
    uint16_t pos_int;
    uint16_t spd_int;
    uint16_t trq_int;
    uint16_t tmp_int;
    float pos_raw;
    float delta;
    RobStrideMotorLimit* limit;

    if (frame == NULL || motor == NULL ||
        frame->canbus_id != motor->can_bus || frame->IDE != 1U ||
        frame->isRemote != 0 || frame->Length < 8U)
        return;

    if (RobStrideMotor_HandleScan(motor, frame))
        return;

    id = frame->Id.all;
    cmd = (id >> ROBSTRIDE_CMD_OFFSET) & 0x1FU;
    sender_id = (id >> ROBSTRIDE_ID_OFFSET) & 0xFFU;

    if ((id & 0xFFU) != motor->mst_id)
        return;

    if (sender_id != (uint8_t)motor->id)
        return;

    limit = &motor->Data.limit;

    if (cmd == RS_CMD_FEEDBACK) {
        motor->Data.rx_mode_state = (RobStrideModeState)((id >> 22) & 0x03U);
        motor->Data.rx_error_code = (uint16_t)((id >> 16) & 0x3FU);
        pos_int = (frame->Data.uchars[0] << 8) | frame->Data.uchars[1];
        spd_int = (frame->Data.uchars[2] << 8) | frame->Data.uchars[3];
        trq_int = (frame->Data.uchars[4] << 8) | frame->Data.uchars[5];
        tmp_int = (frame->Data.uchars[6] << 8) | frame->Data.uchars[7];

        pos_raw = UINT_TO_FLOAT(pos_int, limit->p_min, limit->p_max, ROBSTRIDE_QUANT_BITS);
        if (motor->rx_count == 0U) {
            motor->rx_Pos = pos_raw;
        } else {
            delta = pos_raw - motor->Data.PosRaw;
            if (delta > ROBSTRIDE_POS_HALF_CYCLE)
                delta -= ROBSTRIDE_POS_CYCLE;
            else if (delta < -ROBSTRIDE_POS_HALF_CYCLE)
                delta += ROBSTRIDE_POS_CYCLE;

            motor->rx_Pos += delta;
        }

        motor->Data.PosRaw = pos_raw;
        motor->rx_W = UINT_TO_FLOAT(spd_int, limit->v_min, limit->v_max, ROBSTRIDE_QUANT_BITS);
        motor->rx_T = UINT_TO_FLOAT(trq_int, limit->t_min, limit->t_max, ROBSTRIDE_QUANT_BITS);
        motor->rx_Temp = (float)tmp_int / ROBSTRIDE_TEMP_SCALE;
        motor->rx_count++;
        motor->flag.lost_count = 0U;
        motor->flag.connected = 1U;
    } else if (cmd == RS_CMD_ERROR) {
        /* Type 21 详细故障值为数据区前 4 字节的小端 uint32。 */
        motor->Data.rx_error_code =
            ((uint32_t)frame->Data.uchars[0]) |
            ((uint32_t)frame->Data.uchars[1] << 8) |
            ((uint32_t)frame->Data.uchars[2] << 16) |
            ((uint32_t)frame->Data.uchars[3] << 24);
    }
}

/**
 * @brief 通用 FDCAN 分发器适配入口
 *
 * 灵足将命令类型、发送端 ID 和接收端 ID 编码在 29 位扩展 ID 中。
 * 本函数先完成帧类型和对象寻址检查，只有唯一匹配时才交给协议解析，
 * 因此不会把同一总线上的脉塔/达妙标准帧误当成灵足反馈。
 */
uint8_t RobStrideMotor_RxHandler(const FDCANFrame* frame, void* context) {
    RobStrideMotor* motor = (RobStrideMotor*)context;
    uint32_t id;
    uint8_t cmd;
    uint8_t sender_id;

    if (frame == NULL || motor == NULL ||
        frame->canbus_id != motor->can_bus || frame->IDE != 1U ||
        frame->isRemote != 0 || frame->Length < 8U)
        return 0U;

    id = frame->Id.all & 0x1FFFFFFFU;
    cmd = (uint8_t)((id >> ROBSTRIDE_CMD_OFFSET) & 0x1FU);

    /* 扫描模式的读 ID 回复不能预先知道发送端 ID。 */
    if (motor->Data.work_mode == ROBSTRIDE_WORK_SCAN &&
        cmd == RS_CMD_GET_ID && (id & 0xFFU) == motor->mst_id) {
        Process_RobStride_Motor_Frame(frame, motor);
        return 1U;
    }

    sender_id = (uint8_t)((id >> ROBSTRIDE_ID_OFFSET) & 0xFFU);
    if ((id & 0xFFU) != motor->mst_id ||
        sender_id != (uint8_t)motor->id ||
        (cmd != RS_CMD_FEEDBACK && cmd != RS_CMD_ERROR))
        return 0U;

    Process_RobStride_Motor_Frame(frame, motor);
    return 1U;
}
