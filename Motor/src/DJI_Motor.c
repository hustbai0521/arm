/**
 * @file    DJI_Motor.c
 * @brief   DJI CAN motor driver source file.
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-05
 */

#include "DJI_Motor.h"
#include "FDCAN_Basic.h"
#include "fdcan.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

static void DJIMotor_ConfigType(DJIMotor* dji_motor);
static bool DJIMotor_FeedbackCheck(DJIMotor* motor);
static uint16_t DJIMotor_GetControlStdId(DJICanMotorGroup motor_group);
static void DjiMotor_Transmit(uint16_t id, uint8_t bus_id, uint8_t* data, uint8_t len);

/* ========================== Common motor logic ========================== */

/**
 * @brief  初始化 DJI 电机对象和默认 PID 参数
 * @param  dji_motor: DJI 电机对象指针
 * @param  type:      电机型号
 * @param  can_id:    DJI 电机反馈 ID (1 ~ 8)
 * @retval 无
 */
void DJI_Motor_Init(DJIMotor* dji_motor, DJIMotorType type, uint8_t can_id) {
    if (dji_motor == NULL)
        return;

    memset(dji_motor, 0, sizeof(DJIMotor));

    dji_motor->Motor.ID = can_id;
    dji_motor->Type = type;

    DJIMotor_ConfigType(dji_motor);
}

/**
 * @brief  更新一次控制计算并刷新电流期望
 * @param  DJIMotor: DJI 电机对象指针
 * @retval 无
 */
void DJIMotor_Control_Update(DJIMotor* DJIMotor) {
    if (DJIMotor == NULL)
        return;

    (void)DJIMotor_FeedbackCheck(DJIMotor);

    switch (DJIMotor->Motor.mode) {
        case MOTOR_POSITION:
            DJIMotor->Motor.SpeedTar =
                Pid_Regulate_Auto(DJIMotor->Motor.PosTar, DJIMotor->Motor.PosMea, &DJIMotor->PID.PosPID);

        case MOTOR_SPEED:
            DJIMotor->Motor.CurTar =
                Pid_Regulate_Auto(DJIMotor->Motor.SpeedTar, DJIMotor->Motor.SpeedMea, &DJIMotor->PID.SpeedPID);
            break;

        case MOTOR_CURRENT:
            break;

        case MOTOR_ERROR:
            break;

        default:
            DJIMotor->Motor.CurTar = 0.0f;
            break;
    }
}

/**
 * @brief  通过一帧 CAN 报文发送 4 路 DJI 电机电流命令
 * @param  motor_group: 电机控制分组 (1 ~ 4 或 5 ~ 8)
 * @param  bus_id:      CAN 总线编号
 * @param  current1:    分组内第 1 路电机电流命令 (-16384 ~ 16384)
 * @param  current2:    分组内第 2 路电机电流命令 (-16384 ~ 16384)
 * @param  current3:    分组内第 3 路电机电流命令 (-16384 ~ 16384)
 * @param  current4:    分组内第 4 路电机电流命令 (-16384 ~ 16384)
 * @retval 无
 */
void DJI_Can_Set(DJICanMotorGroup motor_group, uint8_t bus_id, int16_t current1, int16_t current2, int16_t current3,
                 int16_t current4) {
    uint8_t buffer[8];
    uint16_t id = DJIMotor_GetControlStdId(motor_group);

    if (id == 0U)
        return;

    buffer[0] = (uint8_t)((current1 >> 8) & 0xFF);
    buffer[1] = (uint8_t)(current1 & 0xFF);
    buffer[2] = (uint8_t)((current2 >> 8) & 0xFF);
    buffer[3] = (uint8_t)(current2 & 0xFF);
    buffer[4] = (uint8_t)((current3 >> 8) & 0xFF);
    buffer[5] = (uint8_t)(current3 & 0xFF);
    buffer[6] = (uint8_t)((current4 >> 8) & 0xFF);
    buffer[7] = (uint8_t)(current4 & 0xFF);

    DjiMotor_Transmit(id, bus_id, buffer, 8);
}

/**
 * @brief  解析一帧 DJI 电机反馈报文
 * @param  Frame_Process: 接收到的 FDCAN 帧指针
 * @param  DJIMotor:      DJI 电机对象指针
 * @retval 无
 */
void Process_DJI_Frame(FDCANFrame* Frame_Process, DJIMotor* DJIMotor) {
    int distance;
    uint8_t id;
    uint16_t temp_angle;
    int16_t temp_speed;
    int16_t temp_current;

    if (Frame_Process == NULL || DJIMotor == NULL)
        return;

    distance = 0;
    id = (uint8_t)(Frame_Process->Id.StdID & 0x00F);
    temp_angle = (uint16_t)(Frame_Process->Data.uchars[0] << 8 | Frame_Process->Data.uchars[1]);
    temp_speed = (int16_t)(Frame_Process->Data.uchars[2] << 8 | Frame_Process->Data.uchars[3]);
    temp_current = (int16_t)(Frame_Process->Data.uchars[4] << 8 | Frame_Process->Data.uchars[5]);

    if (id != DJIMotor->Motor.ID)
        return;

    DJIMotor->Motor.connected = DJI_MOTOR_RX_ONLINE;
    DJIMotor->Motor.data.rx_count++;
    DJIMotor->Motor.data.lost_count = 0U;

    if (DJIMotor->Motor.PreEncoder == 0U && DJIMotor->Motor.Encoder == 0U) {
        DJIMotor->Motor.PreEncoder = temp_angle;
        DJIMotor->Motor.Encoder = temp_angle;
    } else {
        DJIMotor->Motor.PreEncoder = DJIMotor->Motor.Encoder;
        DJIMotor->Motor.Encoder = temp_angle;
    }

    DJIMotor->Motor.PrePos = DJIMotor->Motor.PosMea;

    distance = (int)DJIMotor->Motor.Encoder - (int)DJIMotor->Motor.PreEncoder;
    if (abs(distance) > DJI_MOTOR_ENCODER_HALF_RANGE) {
        distance = distance - distance / abs(distance) * DJI_MOTOR_ENCODER_RESOLUTION;
    }

    DJIMotor->Motor.PosMea += (float)distance * DJIMotor->Motor.data.PosScale;
    DJIMotor->Motor.SpeedMea = (float)temp_speed * DJIMotor->Motor.data.SpeedScale;
    DJIMotor->Motor.CurMea = (float)temp_current;
}

/* ========================== Common helper functions ========================== */

/**
 * @brief  配置电机型号相关换算系数和默认 PID 参数
 * @param  dji_motor: DJI 电机对象指针
 * @retval 无
 */
static void DJIMotor_ConfigType(DJIMotor* dji_motor) {
    DJIMotor_PID* pid = &dji_motor->PID;

    pid->PosPID.Kp = 5.0f;
    pid->PosPID.Ki = 0.005f;
    pid->PosPID.Kd = 0.10f;
    pid->PosPID.Integral = 0;
    pid->PosPID.LimitIntegral = 0.0f;
    pid->PosPID.LimitOutput = 0.0f;
    pid->PosPID.PreError = 0;

    pid->SpeedPID.Kp = 50.0f;
    pid->SpeedPID.Ki = 0.1f;
    pid->SpeedPID.Kd = 0.0f;
    pid->SpeedPID.Integral = 0;
    pid->SpeedPID.LimitIntegral = 500.0f;
    pid->SpeedPID.PreError = 0;

    switch (dji_motor->Type) {
        case DJI_MOTOR_TYPE_3508:
            dji_motor->Motor.data.PosScale = DJI_MOTOR_3508_POSITION_SCALE;
            dji_motor->Motor.data.SpeedScale = DJI_MOTOR_3508_SPEED_SCALE;
            pid->SpeedPID.LimitOutput = DJI_MOTOR_3508_SPEED_PID_LIMIT;
            break;

        case DJI_MOTOR_TYPE_2006:
        default:
            dji_motor->Motor.data.PosScale = DJI_MOTOR_2006_POSITION_SCALE;
            dji_motor->Motor.data.SpeedScale = DJI_MOTOR_2006_SPEED_SCALE;
            pid->SpeedPID.LimitOutput = DJI_MOTOR_2006_SPEED_PID_LIMIT;
            break;
    }

    memset(&pid->ReservePID, 0, sizeof(PIDStructTypedef));
}

/**
 * @brief  检查 DJI 电机反馈是否在线
 * @param  motor: DJI 电机对象指针
 * @retval true 反馈在线或短暂丢帧仍在容忍范围内
 * @retval false 从未收到反馈或连续丢帧超限
 */
static bool DJIMotor_FeedbackCheck(DJIMotor* motor) {
    if (motor == NULL)
        return false;

    if (motor->Motor.data.rx_count == 0U) {
        motor->Motor.connected = DJI_MOTOR_RX_OFFLINE;
        return false;
    }

    if (motor->Motor.data.rx_count != motor->Motor.data.rx_last) {
        motor->Motor.data.rx_last = motor->Motor.data.rx_count;
        motor->Motor.data.lost_count = 0U;
        motor->Motor.connected = DJI_MOTOR_RX_ONLINE;
        return true;
    }

    if (motor->Motor.data.lost_count < UINT16_MAX)
        motor->Motor.data.lost_count++;

    if (motor->Motor.data.lost_count >= DJI_MOTOR_FEEDBACK_LOST_LIMIT) {
        motor->Motor.connected = DJI_MOTOR_RX_OFFLINE;
        return false;
    }

    return true;
}

/* ========================== Project-specific CAN adaptation ========================== */

/**
 * @brief  将电机分组枚举转换为 DJI 控制帧标准 ID
 * @param  motor_group: 电机控制分组
 * @retval 0x200 (1 ~ 4 号电机)
 * @retval 0x1FF (5 ~ 8 号电机)
 * @retval 0 (无效分组)
 */
static uint16_t DJIMotor_GetControlStdId(DJICanMotorGroup motor_group) {
    switch (motor_group) {
        case DJI_CAN_MOTOR_GROUP_1_4:
            return DJI_CAN_STDID_CMD_1_4;

        case DJI_CAN_MOTOR_GROUP_5_8:
            return DJI_CAN_STDID_CMD_5_8;

        default:
            return 0U;
    }
}

/**
 * @brief  发送一帧 DJI 电机 CAN 报文
 * @param  id:     标准帧 CAN ID
 * @param  bus_id: CAN 总线编号
 * @param  data:   发送数据缓冲区
 * @param  len:    发送数据长度 (字节, 0 ~ 8)
 * @retval 无
 */
static void DjiMotor_Transmit(uint16_t id, uint8_t bus_id, uint8_t* data, uint8_t len) {
#if defined(USE_CAN_1) || defined(USE_CAN_2)
    if (len > 8U)
        len = 8U;

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.Identifier = id;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = len;

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    FDCAN_Transmit_Message(bus_id, &TxHeader, data);
#endif
}
