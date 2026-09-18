#include "Unitree_Motor.h"
#include "CRC.h"
#include "usart.h"
#include <string.h>

/**
 * @brief 限制变量在范围内
 * @param _IN 输入变量 (会被修改)
 * @param _MIN 最小值
 * @param _MAX 最大值
 */
#define SATURATE(_IN, _MIN, _MAX) \
    {                             \
        if ((_IN) <= (_MIN))      \
            (_IN) = (_MIN);       \
        else if ((_IN) >= (_MAX)) \
            (_IN) = (_MAX);       \
    }

static void UnitreeMotor_UpdateConnection(UnitreeMotor* motor);
static void UnitreeMotor_PackCmd(UnitreeMotor* motor);
static void UnitreeMotor_CleanDCacheBuffer(const void* addr, uint32_t size);

/**
 * @brief 电机数据结构初始化
 * @param motor 电机对象指针
 * @param id 电机ID
 * @param type 电机类型
 */
void UnitreeMotor_Init(UnitreeMotor* motor, UnitreeMotorType type, uint8_t id) {
    if (motor == NULL)
        return;

    memset(motor, 0, sizeof(UnitreeMotor));

    motor->id = id;
    motor->type = type;

    switch (type) {
        case UNITREE_MOTOR_TYPE_GO1:
            motor->param_scale.tx_msg_len = sizeof(UnitreeGo_ControlData);
            motor->param_scale.rx_msg_len = sizeof(UnitreeGo_MotorData);
            motor->param_scale.rx_head[0] = 0xFD;
            motor->param_scale.rx_head[1] = 0xEE;

            motor->param_scale.kp_scale = UNITREE_GO1_KP_SCALE;
            motor->param_scale.kw_scale = UNITREE_GO1_KW_SCALE;
            motor->param_scale.pos_scale = UNITREE_GO1_POS_SCALE;
            motor->param_scale.spd_scale = UNITREE_GO1_SPD_SCALE;
            motor->param_scale.tor_scale = UNITREE_GO1_TORQUE_SCALE;
            motor->param_scale.tor_limit = UNITREE_GO1_LIMIT_TORQUE;
            motor->param_scale.spd_limit = UNITREE_GO1_LIMIT_SPEED;
            break;

        case UNITREE_MOTOR_TYPE_A1:
            motor->param_scale.tx_msg_len = sizeof(UnitreeA1_ControlData);
            motor->param_scale.rx_msg_len = sizeof(UnitreeA1_MotorData);
            motor->param_scale.rx_head[0] = 0xFE;
            motor->param_scale.rx_head[1] = 0xEE;

            motor->param_scale.kp_scale = UNITREE_A1_KP_SCALE;
            motor->param_scale.kw_scale = UNITREE_A1_KW_SCALE;
            motor->param_scale.pos_scale = UNITREE_A1_POS_SCALE;
            motor->param_scale.spd_scale = UNITREE_A1_SPD_SCALE;
            motor->param_scale.tor_scale = UNITREE_A1_TORQUE_SCALE;
            motor->param_scale.tor_limit = UNITREE_A1_LIMIT_TORQUE;
            motor->param_scale.spd_limit = UNITREE_A1_LIMIT_SPEED;
            break;

        default:
            break;
    }

    motor->fbk_rx_index = 0;

    UnitreeMotor_PackCmd(motor);
}

void UnitreeMotor_Control_Update(UnitreeMotor* motor) {
    if (motor == NULL)
        return;

    if (motor->flag.fbk_ready_flag) {
        __DMB();
        UnitreeMotor_UnpackFbk(motor);
        __DMB();
        motor->flag.fbk_ready_flag = 0;
    }

    UnitreeMotor_UpdateConnection(motor);

    switch (motor->ctrl_mode) {
        case UNITREE_CTRL_POS_SPD:
            motor->W = Pid_Regulate_Auto(motor->Pos, motor->rx_Pos, &motor->PID.PosPID);
            motor->T = Pid_Regulate_Auto(motor->W, motor->rx_W, &motor->PID.SpeedPID);
            break;

        case UNITREE_CTRL_POS:
            motor->W = Pid_Regulate_Auto(motor->Pos, motor->rx_Pos, &motor->PID.PosPID);
            break;

        case UNITREE_CTRL_NATIVE:
            break;

        case UNITREE_CTRL_STOP:
        default:
            break;
    }
}

/**
 * @brief 统一发送电机控制指令
 * @param motor 电机对象指针
 * @param huart 发送使用的串口句柄
 */
void UnitreeMotor_Transmit(UnitreeMotor* motor, UART_HandleTypeDef* huart) {
    if (motor == NULL || huart == NULL)
        return;

    if (motor->flag.one_shot_flag == 2U)
        return;

    // __HAL_DMA_DISABLE(huart->hdmarx);
    // __HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));

    UnitreeMotor_PackCmd(motor);

    HAL_UART_Transmit(huart, (uint8_t*)&motor->cmd_buffer, motor->param_scale.tx_msg_len, 10);

    motor->tx_count++;

    if (motor->flag.one_shot_flag == 1U) {
        motor->flag.one_shot_flag = 2U;
    }
}

void UnitreeMotor_Transmit_DMA(UnitreeMotor* motor, UART_HandleTypeDef* huart) {
    HAL_StatusTypeDef status;

    if (motor == NULL || huart == NULL)
        return;

    if (motor->flag.one_shot_flag == 2U)
        return;

    UnitreeMotor_PackCmd(motor);

    if (huart->hdmatx == NULL) {
        status = HAL_UART_Transmit(huart, (uint8_t*)&motor->cmd_buffer, motor->param_scale.tx_msg_len, 10U);
    } else {
        UnitreeMotor_CleanDCacheBuffer(motor->cmd_buffer, motor->param_scale.tx_msg_len);
        status = HAL_UART_Transmit_DMA(huart, (uint8_t*)&motor->cmd_buffer, motor->param_scale.tx_msg_len);
    }

    if (status != HAL_OK)
        return;

    motor->tx_count++;

    if (motor->flag.one_shot_flag == 1U) {
        motor->flag.one_shot_flag = 2U;
    }
}

/**
 * @brief 解析电机反馈数据
 * @param motor 电机对象指针 (需确保 rx_buffer 已填充最新数据)
 */
void UnitreeMotor_UnpackFbk(UnitreeMotor* motor) {
    if (motor == NULL)
        return;

    switch (motor->type) {
        case UNITREE_MOTOR_TYPE_GO1: {
            UnitreeGo_MotorData* pFbk = (UnitreeGo_MotorData*)motor->rx_buffer;
            uint16_t calc_crc = crc_ccitt(0, (uint8_t*)pFbk, sizeof(UnitreeGo_MotorData) - sizeof(pFbk->CRC16));

            if (pFbk->CRC16 != calc_crc) {
                motor->rx_error = UNITREE_RX_ERROR_CRC_MISMATCH;
                return;
            }

            motor->flag.connected = 1U;
            motor->flag.lost_count = 0U;
            motor->rx_count++;
            motor->rx_error = UNITREE_RX_ERROR_NONE;

            motor->rx_Temp = pFbk->temp;
            motor->rx_MError = pFbk->MError;
            motor->rx_footForce = pFbk->force;
            motor->rx_W = (float)pFbk->speed / motor->param_scale.spd_scale;
            motor->rx_T = (float)pFbk->torque / motor->param_scale.tor_scale;
            motor->rx_Pos = (float)pFbk->pos / motor->param_scale.pos_scale;
            break;
        }

        case UNITREE_MOTOR_TYPE_A1: {
            UnitreeA1_MotorData* pFbk = (UnitreeA1_MotorData*)motor->rx_buffer;
            uint32_t calc_crc = crc32_core((uint32_t*)pFbk, 18);

            if (pFbk->CRC32 != calc_crc) {
                motor->rx_error = UNITREE_RX_ERROR_CRC_MISMATCH;
                return;
            }

            motor->flag.connected = 1U;
            motor->flag.lost_count = 0U;
            motor->rx_count++;
            motor->rx_error = UNITREE_RX_ERROR_NONE;

            motor->rx_Temp = pFbk->Temp;
            motor->rx_MError = pFbk->MError;
            motor->rx_footForce = pFbk->Force16;
            motor->rx_W = (float)pFbk->W / motor->param_scale.spd_scale;
            motor->rx_T = (float)pFbk->T / motor->param_scale.tor_scale;
            motor->rx_Pos = (float)pFbk->Pos / motor->param_scale.pos_scale;
            break;
        }

        default:
            motor->rx_error = UNITREE_RX_ERROR_TYPE_UNSUPPORT;
            break;
    }
}

/**
 * @brief 宇树电机协议流解析 (字节对齐状态机)
 * @param motor 电机对象
 * @param byte 单字节数据
 */
void Process_Unitree_Motor_Byte(UnitreeMotor* motor, uint8_t byte) {
    uint8_t* p_stream;

    if (motor == NULL)
        return;

    /* 任务尚未消费上一帧时不覆盖已发布的反馈缓冲区。 */
    if (motor->flag.fbk_ready_flag != 0U)
        return;

    if (motor->flag.rx_ignore_size > 0U) {
        motor->flag.rx_ignore_size--;
        return;
    }

    p_stream = motor->rx_buffer;

    switch (motor->fbk_rx_index) {
        case 0:
            if (byte == motor->param_scale.rx_head[0]) {
                p_stream[motor->fbk_rx_index++] = byte;
            }
            break;

        case 1:
            if (byte == motor->param_scale.rx_head[1]) {
                p_stream[motor->fbk_rx_index++] = byte;
            } else {
                motor->rx_error = UNITREE_RX_ERROR_HEADER_2;
                motor->fbk_rx_index = (byte == motor->param_scale.rx_head[0]) ? 1 : 0;
            }
            break;

        case 2: {
            uint8_t rx_id_val;
            uint8_t frame_len;

            p_stream[motor->fbk_rx_index++] = byte;
            rx_id_val = byte;
            frame_len = motor->param_scale.rx_msg_len;

            if (motor->type == UNITREE_MOTOR_TYPE_GO1)
                rx_id_val &= 0x0F;

            if (rx_id_val != motor->id) {
                motor->rx_error = UNITREE_RX_ERROR_ID_MISMATCH;
                if (frame_len < 3U || frame_len > UNITREE_PACKET_MAX_LEN) {
                    motor->rx_error = UNITREE_RX_ERROR_FRAME_OVERFLOW;
                } else {
                    motor->flag.rx_ignore_size = (uint8_t)(frame_len - 3U);
                }
                motor->fbk_rx_index = 0;
            }
            break;
        }

        default:
            if (motor->param_scale.rx_msg_len == 0U || motor->param_scale.rx_msg_len > UNITREE_PACKET_MAX_LEN) {
                motor->rx_error = UNITREE_RX_ERROR_FRAME_OVERFLOW;
                motor->fbk_rx_index = 0;
                break;
            }

            p_stream[motor->fbk_rx_index++] = byte;

            if (motor->fbk_rx_index >= motor->param_scale.rx_msg_len) {
                __DMB();
                motor->flag.fbk_ready_flag = 1U;
                motor->fbk_rx_index = 0;
            }
            break;
    }
}

// 根据有效反馈计数判断通讯是否仍在线，不依赖系统时间戳。
static void UnitreeMotor_UpdateConnection(UnitreeMotor* motor) {
    if (motor == NULL)
        return;

    if (motor->rx_count == 0U) {
        motor->flag.connected = 0U;
        if (motor->flag.lost_count < UINT16_MAX)
            motor->flag.lost_count++;
        if (motor->flag.lost_count >= UNITREE_FEEDBACK_LOST_LIMIT &&
            motor->rx_error == UNITREE_RX_ERROR_NONE) {
            motor->rx_error = UNITREE_RX_ERROR_LOST;
        }
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

    if (motor->flag.lost_count >= UNITREE_FEEDBACK_LOST_LIMIT) {
        motor->flag.connected = 0U;
        if (motor->rx_error == UNITREE_RX_ERROR_NONE)
            motor->rx_error = UNITREE_RX_ERROR_LOST;
    }
}

static void UnitreeMotor_CleanDCacheBuffer(const void* addr, uint32_t size) {
    uint32_t start = (uint32_t)addr & ~31U;
    uint32_t end = ((uint32_t)addr + size + 31U) & ~31U;

    SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
}

/**
 * @brief 将发送给电机的浮点参数转换为定点类型参数
 */
static void UnitreeMotor_PackCmd(UnitreeMotor* motor) {
    float kp = 0.0f, kw = 0.0f, pos = 0.0f, spd = 0.0f, tau = 0.0f;
    uint8_t mode = 0U;
    uint8_t id;

    if (motor == NULL)
        return;

    id = motor->id;

    switch (motor->ctrl_mode) {
        case UNITREE_CTRL_NATIVE:
            mode = 1U;
            if (motor->flag.safe_switch) {
                motor->W = 0.0f;
                motor->T = 0.0f;
                motor->Pos = motor->rx_Pos;
                motor->flag.safe_switch = 0U;
            }

            kp = motor->K_P;
            kw = motor->K_W;
            pos = motor->Pos;
            spd = motor->W;
            tau = motor->T;
            break;

        case UNITREE_CTRL_POS:
            mode = 1U;
            kp = 0.0f;
            kw = motor->K_W;
            pos = 0.0f;
            spd = motor->W;
            tau = 0.0f;
            motor->flag.safe_switch = 1U;
            break;

        case UNITREE_CTRL_POS_SPD:
            mode = 1U;
            kp = 0.0f;
            kw = 0.0f;
            pos = 0.0f;
            spd = 0.0f;
            tau = motor->T;
            motor->flag.safe_switch = 1U;
            break;

        default:
            mode = 0U;
            break;
    }

    SATURATE(tau, -motor->param_scale.tor_limit, motor->param_scale.tor_limit);
    SATURATE(spd, -motor->param_scale.spd_limit, motor->param_scale.spd_limit);

    switch (motor->type) {
        case UNITREE_MOTOR_TYPE_GO1: {
            UnitreeGo_ControlData* pCmd = (UnitreeGo_ControlData*)motor->cmd_buffer;
            memset(pCmd, 0, sizeof(UnitreeGo_ControlData));

            pCmd->head[0] = 0xFE;
            pCmd->head[1] = 0xEE;
            pCmd->id = id;
            pCmd->status = mode;
            pCmd->K_P = (int16_t)(kp * motor->param_scale.kp_scale);
            pCmd->K_W = (int16_t)(kw * motor->param_scale.kw_scale);
            pCmd->Pos = (int32_t)(pos * motor->param_scale.pos_scale);
            pCmd->W = (int16_t)(spd * motor->param_scale.spd_scale);
            pCmd->T = (int16_t)(tau * motor->param_scale.tor_scale);
            pCmd->CRC16 = crc_ccitt(0, (uint8_t*)pCmd, sizeof(UnitreeGo_ControlData) - sizeof(pCmd->CRC16));
            break;
        }

        case UNITREE_MOTOR_TYPE_A1: {
            UnitreeA1_ControlData* pCmd = (UnitreeA1_ControlData*)motor->cmd_buffer;
            memset(pCmd, 0, sizeof(UnitreeA1_ControlData));

            pCmd->head[0] = 0xFE;
            pCmd->head[1] = 0xEE;
            pCmd->id = id;
            pCmd->status = (mode == 0U) ? 0 : 10;
            pCmd->ModifyBit = 0xFF;
            pCmd->ReadBit = 0x00;
            pCmd->K_P = (int16_t)(kp * motor->param_scale.kp_scale);
            pCmd->K_W = (int16_t)(kw * motor->param_scale.kw_scale);
            pCmd->Pos = (int32_t)(pos * motor->param_scale.pos_scale);
            pCmd->W = (int16_t)(spd * motor->param_scale.spd_scale);
            pCmd->T = (int16_t)(tau * motor->param_scale.tor_scale);
            pCmd->CRC32 = crc32_core((uint32_t*)pCmd, 7);
            break;
        }

        default:
            break;
    }
}
