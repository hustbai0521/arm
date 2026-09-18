/**
 * @file    DJI_Motor.h
 * @brief   DJI CAN motor driver interface for feedback parsing and current control.
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-05
 */

#ifndef _DJI_MOTOR_H
#define _DJI_MOTOR_H

#include "FDCAN_Basic.h"
#include "PID.h"
#include "math.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief DJI 电机协议常量
 */
#define DJI_MOTOR_ENCODER_RESOLUTION 8192                                ///< 编码器单圈计数值
#define DJI_MOTOR_ENCODER_HALF_RANGE (DJI_MOTOR_ENCODER_RESOLUTION / 2)  ///< 编码器半圈范围，用于过零判断
#define DJI_CAN_STDID_CMD_1_4 0x200U                                     ///< 1 ~ 4 号电机电流控制帧标准 ID
#define DJI_CAN_STDID_CMD_5_8 0x1FFU                                     ///< 5 ~ 8 号电机电流控制帧标准 ID
#define DJI_MOTOR_FEEDBACK_LOST_LIMIT 1000U                              ///< 默认连续反馈丢失检测次数

/**
 * @brief DJI 电机型号换算常量
 */
#define DJI_MOTOR_3508_REDUCTION_RATIO (3591.0f / 187.0f)  ///< M3508 减速比
#define DJI_MOTOR_2006_REDUCTION_RATIO 36.0f               ///< M2006 减速比
#define DJI_MOTOR_3508_POSITION_SCALE \
    (360.0f / DJI_MOTOR_3508_REDUCTION_RATIO / DJI_MOTOR_ENCODER_RESOLUTION)  ///< M3508 编码器增量转输出轴角度系数 (deg)
#define DJI_MOTOR_3508_SPEED_SCALE (1.0f / DJI_MOTOR_3508_REDUCTION_RATIO)    ///< M3508 转子转速转输出轴转速系数
#define DJI_MOTOR_3508_SPEED_PID_LIMIT 16000.0f                               ///< M3508 速度环 PID 输出限幅
#define DJI_MOTOR_2006_POSITION_SCALE \
    (360.0f / DJI_MOTOR_2006_REDUCTION_RATIO / DJI_MOTOR_ENCODER_RESOLUTION)  ///< M2006 编码器增量转输出轴角度系数 (deg)
#define DJI_MOTOR_2006_SPEED_SCALE (1.0f / DJI_MOTOR_2006_REDUCTION_RATIO)    ///< M2006 转子转速转输出轴转速系数
#define DJI_MOTOR_2006_SPEED_PID_LIMIT 8000.0f                                ///< M2006 速度环 PID 输出限幅

/**
 * @brief DJI 电机控制模式
 */
typedef enum {
    MOTOR_IDLE,      ///< 空闲模式，控制更新时清零电流输出
    MOTOR_POSITION,  ///< 位置模式，位置环输出速度期望
    MOTOR_SPEED,     ///< 速度模式，速度环输出电流期望
    MOTOR_CURRENT,   ///< 电流模式，直接使用外部电流期望
    MOTOR_ERROR,     ///< 预留错误状态
} DJIMotorMode;

/**
 * @brief DJI CAN 电流控制帧电机分组
 */
typedef enum {
    DJI_CAN_MOTOR_GROUP_1_4 = DJI_CAN_STDID_CMD_1_4,  ///< 1 ~ 4 号电机，控制帧 ID 为 0x200
    DJI_CAN_MOTOR_GROUP_5_8 = DJI_CAN_STDID_CMD_5_8,  ///< 5 ~ 8 号电机，控制帧 ID 为 0x1FF
} DJICanMotorGroup;

/**
 * @brief DJI 电机反馈状态
 */
typedef enum {
    DJI_MOTOR_RX_OFFLINE = 0,  ///< 反馈离线
    DJI_MOTOR_RX_ONLINE,       ///< 反馈在线
} DJIMotorRxState;

/**
 * @brief DJI 电机运行状态和反馈量
 */
typedef struct
{
    uint8_t ID;         ///< DJI 电机反馈 ID (1 ~ 8)
    DJIMotorMode mode;  ///< 当前控制模式

    float PosTar;    ///< 输出轴目标位置 (deg)
    float PosMea;    ///< 输出轴累计测量位置 (deg)
    float SpeedTar;  ///< 输出轴目标转速 (rpm)
    float SpeedMea;  ///< 输出轴测量转速 (rpm)
    float CurTar;    ///< 目标电流命令 (-16384 ~ 16384)
    float CurMea;    ///< 反馈电流值

    uint16_t Encoder;     ///< 当前原始编码器值 (0 ~ 8191)
    uint16_t PreEncoder;  ///< 上一次原始编码器值 (0 ~ 8191)
    float PrePos;         ///< 上一次输出轴测量位置 (deg)

    DJIMotorRxState connected;  ///< 反馈在线状态

    struct
    {
        float PosScale;                ///< 编码器增量转输出轴角度系数
        float SpeedScale;              ///< 转子转速转输出轴转速系数
        volatile uint32_t rx_count;    ///< 有效反馈帧累计数量
        uint32_t rx_last;              ///< 上一次检测时的反馈帧数量
        volatile uint16_t lost_count;  ///< 连续未收到新反馈的检测次数
    } data;

} DJIMotorState;

/**
 * @brief DJI 电机 PID 控制器
 */
typedef struct
{
    PIDStructTypedef SpeedPID;    ///< 速度环 PID
    PIDStructTypedef PosPID;      ///< 位置环 PID
    PIDStructTypedef ReservePID;  ///< 预留 PID
} DJIMotor_PID;

/**
 * @brief 支持的 DJI 电机型号
 */
typedef enum {
    DJI_MOTOR_TYPE_3508,  ///< DJI M3508 电机
    DJI_MOTOR_TYPE_2006   ///< DJI M2006 电机
} DJIMotorType;

/**
 * @brief DJI 电机对象
 */
typedef struct
{
    DJIMotorState Motor;  ///< 运行状态
    DJIMotor_PID PID;     ///< PID 控制器
    DJIMotorType Type;    ///< 电机型号
} DJIMotor;

/**
 * @brief  更新一次控制计算并刷新电流期望
 * @param  DJIMotor: DJI 电机对象指针
 * @retval 无
 */
void DJIMotor_Control_Update(DJIMotor* DJIMotor);

/**
 * @brief  初始化 DJI 电机对象和默认 PID 参数
 * @param  dji_motor: DJI 电机对象指针
 * @param  type:      电机型号
 * @param  can_id:    DJI 电机反馈 ID (1 ~ 8)
 * @retval 无
 */
void DJI_Motor_Init(DJIMotor* dji_motor, DJIMotorType type, uint8_t can_id);

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
                 int16_t current4);

/**
 * @brief  解析一帧 DJI 电机反馈报文
 * @param  Frame_Process: 接收到的 FDCAN 帧指针
 * @param  DJIMotor:      DJI 电机对象指针
 * @retval 无
 */
void Process_DJI_Frame(FDCANFrame* Frame_Process, DJIMotor* DJIMotor);

/*
 * ================== 调用指南 ==================
 * 1. 定义电机对象
 *    DJIMotor motor;
 *
 * 2. 初始化并配置 PID
 *    DJI_Motor_Init(&motor, DJI_MOTOR_TYPE_2006, 3);
 *    然后设置 motor.PID.PosPID / motor.PID.SpeedPID 参数。
 *
 * 3. 在 CAN 接收回调中反馈
 *    Process_DJI_Frame(&frame, &motor);
 *
 * 4. 设定目标模式与目标值
 *    位置模式:
 *    motor.Motor.mode = MOTOR_POSITION;
 *    motor.Motor.PosTar = target_pos;
 *
 *    速度模式:
 *    motor.Motor.mode = MOTOR_SPEED;
 *    motor.Motor.SpeedTar = target_speed;
 *
 *    电流模式:
 *    motor.Motor.mode = MOTOR_CURRENT;
 *    motor.Motor.CurTar = target_current;
 *
 * 5. 在控制周期内调用
 *    DJIMotor_Control_Update(&motor);
 *
 * 6. 将 4 路电流一起通过 DJI_Can_Set(...) 发出
 *    DJI_Can_Set(DJI_CAN_MOTOR_GROUP_1_4, 1, cur1, cur2, cur3, cur4);
 */

#endif
