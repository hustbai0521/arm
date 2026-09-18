/**
 * @file    DM_Motor.h
 * @brief   达妙电机FDCAN驱动头文件
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-06
 */
#ifndef _DM_MOTOR_H_
#define _DM_MOTOR_H_

/* 达妙电机控制通讯时的主设备ID (Master ID) */
#define DAMIAO_MASTER_ID 0x00U
#define DAMIAO_CAN_BITRATE_BPS 1000000UL

#include "FDCAN_Basic.h"
#include "PID.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* 达妙 DM4310 默认配置参数阈值 (依据达妙串口助手中实际配置为准，通常默认值如下) */
#define DM4310_P_MIN (-12.5f)
#define DM4310_P_MAX (12.5f)
#define DM4310_V_MIN (-45.0f)
#define DM4310_V_MAX (45.0f)
#define DM4310_T_MIN (-18.0f)
#define DM4310_T_MAX (18.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)

/*
 * DM-G6220 出厂常用 MIT 映射范围。P/V/T 映射参数可由达妙
 * 调试助手修改，使用前必须与电机内部 PMAX/VMAX/TMAX 保持一致。
 * 注意：协议 TMAX=10 Nm 是量化映射范围，不是允许长时间输出的力矩。
 */
#define DM_G6220_P_MIN (-12.5f)
#define DM_G6220_P_MAX (12.5f)
#define DM_G6220_V_MIN (-45.0f)
#define DM_G6220_V_MAX (45.0f)
#define DM_G6220_T_MIN (-10.0f)
#define DM_G6220_T_MAX (10.0f)
#define DM_G6220_KP_MIN (0.0f)
#define DM_G6220_KP_MAX (500.0f)
#define DM_G6220_KD_MIN (0.0f)
#define DM_G6220_KD_MAX (5.0f)

/* DM-G6220 24 V 版型号参数（仅作选型/保护信息）。 */
#define DM_G6220_RATED_VOLTAGE_V (24.0f)
#define DM_G6220_RATED_CURRENT_A (2.3f)
#define DM_G6220_PEAK_CURRENT_A (5.3f)
#define DM_G6220_RATED_TORQUE_NM (1.3f)
#define DM_G6220_PEAK_TORQUE_NM (2.7f)
#define DM_G6220_RATED_SPEED_RPM (110.0f)
#define DM_G6220_MAX_NO_LOAD_SPEED_RPM (300.0f)
#define DM_G6220_REDUCTION_RATIO (1.0f)
#define DM_G6220_POLE_PAIRS 14U
#define DM_G6220_PHASE_INDUCTANCE_UH (2900.0f)
#define DM_G6220_PHASE_RESISTANCE_OHM (3.5f)
#define DM_G6220_TUNING_UART_BAUDRATE 921600UL

/*
 * DM-J4340-2EC 出厂常用 MIT 映射范围。
 * 这些参数可由达妙调试助手修改，使用前应确保与电机内部 PMAX/VMAX/TMAX 一致。
 */
#define DM_J4340_2EC_P_MIN (-12.5f)
#define DM_J4340_2EC_P_MAX (12.5f)
#define DM_J4340_2EC_V_MIN (-8.0f)
#define DM_J4340_2EC_V_MAX (8.0f)
#define DM_J4340_2EC_T_MIN (-28.0f)
#define DM_J4340_2EC_T_MAX (28.0f)
#define DM_J4340_2EC_KP_MIN (0.0f)
#define DM_J4340_2EC_KP_MAX (500.0f)
#define DM_J4340_2EC_KD_MIN (0.0f)
#define DM_J4340_2EC_KD_MAX (5.0f)

#define DAMIAO_FEEDBACK_LOST_LIMIT 1000U

typedef enum {
    DAMIAO_MOTOR_TYPE_UNKNOWN = 0,
    DAMIAO_MOTOR_TYPE_DM4310,
    DAMIAO_MOTOR_TYPE_DM_J4340_2EC,
    DAMIAO_MOTOR_TYPE_DM_G6220,
    DAMIAO_MOTOR_TYPE_COUNT,
} DamiaoMotorType;

typedef enum {
    DAMIAO_CTRL_STOP = 0,
    DAMIAO_CTRL_MIT,
    DAMIAO_CTRL_MIT_POS,
    DAMIAO_CTRL_MIT_POS_SPD,
    DAMIAO_CTRL_POS,
    DAMIAO_CTRL_SPD,
    DAMIAO_CTRL_PSI,
} DamiaoMotorControlMode;

typedef enum {
    DAMIAO_STATE_DISABLED = 0x0U,        // 未使能
    DAMIAO_STATE_ENABLED = 0x1U,         // 使能成功
    DAMIAO_STATE_OVERVOLTAGE = 0x8U,     // 过压
    DAMIAO_STATE_UNDERVOLTAGE = 0x9U,    // 欠压
    DAMIAO_STATE_OVERCURRENT = 0xAU,     // 过流
    DAMIAO_STATE_MOS_OVER_TEMP = 0xBU,   // MOS 过温
    DAMIAO_STATE_COIL_OVER_TEMP = 0xCU,  // 线圈过温
    DAMIAO_STATE_COMM_LOSS = 0xDU,       // 通信丢失
    DAMIAO_STATE_OVERLOAD = 0xEU,        // 过载
} DamiaoMotorState;

typedef struct
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float t_min;
    float t_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} DamiaoMotorLimit;

/**
 * @brief 型号参数表项。
 * @note  limit 用于 CAN 协议量化/反量化；其它字段是名牌参数。
 *        未收录的名牌参数为 0，不应被解释为物理限值。
 */
typedef struct
{
    DamiaoMotorType type;
    DamiaoMotorLimit limit;
    float rated_voltage_v;
    float rated_current_a;
    float peak_current_a;
    float rated_torque_nm;
    float peak_torque_nm;
    float rated_speed_rpm;
    float max_no_load_speed_rpm;
    float reduction_ratio;
    float phase_inductance_uh;
    float phase_resistance_ohm;
    uint32_t can_bitrate_bps;
    uint32_t tuning_uart_baudrate;
    uint16_t supported_control_modes;
    uint8_t pole_pairs;
} DamiaoMotorModelParameters;

typedef struct
{
    uint8_t connected : 1;
    uint8_t feedback_ready : 1;
    uint8_t safe_switch : 1;
    uint16_t lost_count;
    uint32_t rx_count_last;
} DamiaoMotorFlag;

typedef struct
{
    PIDStructTypedef SpeedPID;
    PIDStructTypedef PosPID;
} DamiaoMotor_PID;

typedef struct
{
    float PosRaw;
} DamiaoMotorData;

typedef struct
{
    uint16_t id;
    uint16_t mst_id;
    uint8_t can_bus;
    DamiaoMotorType type;
    const DamiaoMotorModelParameters* model;
    DamiaoMotorControlMode ctrl_mode;

    float Pos;
    float W;
    float T;
    float K_P;
    float K_W;
    float Current;

    float rx_Pos;
    float rx_W;
    float rx_T;
    float rx_Tmos;
    float rx_Tcoil;
    DamiaoMotorState connected;
    DamiaoMotorData Data;

    DamiaoMotorFlag flag;
    DamiaoMotorLimit limit;
    DamiaoMotor_PID PID;

    uint32_t tx_count;
    uint32_t rx_count;
} DamiaoMotor;

void DamiaoMotor_Init(DamiaoMotor* motor, DamiaoMotorType type, uint8_t can_bus, uint16_t id);
const DamiaoMotorModelParameters* DamiaoMotor_GetModelParameters(DamiaoMotorType type);
uint8_t DamiaoMotor_IsTypeSupported(DamiaoMotorType type);
uint8_t DamiaoMotor_IsControlModeSupported(const DamiaoMotor* motor,
                                           DamiaoMotorControlMode mode);

void DamiaoMotor_Control_Update(DamiaoMotor* motor);
void DamiaoMotor_Enable(DamiaoMotor* motor);
void DamiaoMotor_Disable(DamiaoMotor* motor);
void DamiaoMotor_ClearError(DamiaoMotor* motor);
void DamiaoMotor_SaveZeroPosition(DamiaoMotor* motor);
void DamiaoMotor_ClearCommand(DamiaoMotor* motor);
void DamiaoMotor_Transmit(DamiaoMotor* motor);

void DamiaoMotor_UnpackFeedback(DamiaoMotor* motor, const uint8_t* data, uint8_t len);
void Process_Damiao_Motor_Frame(FDCANFrame* frame, DamiaoMotor* motor);
uint8_t DamiaoMotor_RxHandler(const FDCANFrame* frame, void* context);

/*
 * ================== 调用指南 ==================
 * 1. 定义电机对象
 *    DamiaoMotor motor;
 *
 * 2. 初始化并配置参数
 *    DamiaoMotor_Init(&motor, DAMIAO_MOTOR_TYPE_DM4310, 2, 0x01);
 *    然后设置 motor.K_P / motor.K_W / motor.PID.PosPID / motor.PID.SpeedPID。
 *
 * 3. 在 CAN 接收回调中喂反馈
 *    Process_Damiao_Motor_Frame(&frame, &motor);
 *
 * 4. 选择控制模式并写目标
 *    原生 MIT:
 *    motor.ctrl_mode = DAMIAO_CTRL_MIT;
 *    motor.Pos = target_pos;
 *    motor.W   = target_speed;
 *    motor.T   = target_torque;
 *
 *    软件位置环输出 W:
 *    motor.ctrl_mode = DAMIAO_CTRL_MIT_POS;
 *    motor.Pos = target_pos;
 *
 *    软件位置速度串级输出 T:
 *    motor.ctrl_mode = DAMIAO_CTRL_MIT_POS_SPD;
 *    motor.Pos = target_pos;
 *
 *    位置 / 速度 / PSI 模式:
 *    motor.ctrl_mode = DAMIAO_CTRL_POS / DAMIAO_CTRL_SPD / DAMIAO_CTRL_PSI;
 *
 * 5. 在控制周期内调用
 *    DamiaoMotor_Control_Update(&motor);
 *
 * 6. 周期发送
 *    DamiaoMotor_Transmit(&motor);
 */

#endif
