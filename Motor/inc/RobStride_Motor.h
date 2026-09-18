/**
 * @file    RobStride_Motor.h
 * @brief   RobStride motor driver interface
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-04-26
 */
#ifndef _ROBSTRIDE_MOTOR_H_
#define _ROBSTRIDE_MOTOR_H_

#include "FDCAN_Basic.h"
#include "PID.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief RobStride 通讯常量
 */
#define ROBSTRIDE_MASTER_ID 0xFDU        ///< 主控 ID
#define ROBSTRIDE_BROADCAST_ID 0x00U     ///< 广播 ID
#define ROBSTRIDE_MAX_MOTOR_ID 127U      ///< 最大电机 ID
#define ROBSTRIDE_SCAN_ERROR_ID 0xFFFFU  ///< 扫描失败返回值

/**
 * @brief RobStride 协议常量
 */
#define ROBSTRIDE_QUANT_BITS 16      ///< 量化位宽
#define ROBSTRIDE_CMD_OFFSET 24      ///< 命令字段偏移
#define ROBSTRIDE_ID_OFFSET 8        ///< ID 字段偏移
#define ROBSTRIDE_TEMP_SCALE 10.0f   ///< 温度缩放系数
#define ROBSTRIDE_SCAN_MAX_ROUND 3U  ///< 最大扫描轮数
#define ROBSTRIDE_SCAN_MAX_HIT 3U    ///< 最大确认命中次数

#define ROBSTRIDE_FEEDBACK_LOST_LIMIT 1000U  ///< 反馈丢失阈值

/**
 * @brief RobStride 默认物理限制
 */
#define RS_LIMIT_P_MIN -12.566f
#define RS_LIMIT_P_MAX 12.566f
#define RS_LIMIT_V_MIN -33.0f
#define RS_LIMIT_V_MAX 33.0f
#define RS_LIMIT_T_MIN -14.0f
#define RS_LIMIT_T_MAX 14.0f
#define RS01_LIMIT_V_MIN -44.0f
#define RS01_LIMIT_V_MAX 44.0f
#define RS01_LIMIT_T_MIN -17.0f
#define RS01_LIMIT_T_MAX 17.0f
#define RS05_LIMIT_V_MIN -50.0f
#define RS05_LIMIT_V_MAX 50.0f
#define RS05_LIMIT_T_MIN -5.5f
#define RS05_LIMIT_T_MAX 5.5f
#define EL05_LIMIT_V_MIN -50.0f
#define EL05_LIMIT_V_MAX 50.0f
#define EL05_LIMIT_T_MIN -6.0f
#define EL05_LIMIT_T_MAX 6.0f
#define RS_LIMIT_KP_MIN 0.0f
#define RS_LIMIT_KP_MAX 500.0f
#define RS_LIMIT_KD_MIN 0.0f
#define RS_LIMIT_KD_MAX 5.0f

/**
 * @brief RobStride 控制命令类型
 */
#define RS_CMD_GET_ID 0x00U
#define RS_CMD_CONTROL 0x01U
#define RS_CMD_FEEDBACK 0x02U
#define RS_CMD_ENABLE 0x03U
#define RS_CMD_DISABLE 0x04U
#define RS_CMD_SET_ZERO 0x06U
#define RS_CMD_SET_ID 0x07U
#define RS_CMD_READ_PARAM 0x11U
#define RS_CMD_WRITE_PARAM 0x12U
#define RS_CMD_ERROR 0x15U

/**
 * @brief RobStride RAM 参数索引
 */
#define RS_PARAM_RUN_MODE 0x7005U
#define RS_PARAM_IQ_REF 0x7006U
#define RS_PARAM_SPD_REF 0x700AU
#define RS_PARAM_LIMIT_TRQ 0x700BU
#define RS_PARAM_CUR_KP 0x7010U
#define RS_PARAM_CUR_KI 0x7011U
#define RS_PARAM_LOC_REF 0x7016U
#define RS_PARAM_LIMIT_SPD 0x7017U
#define RS_PARAM_LIMIT_CUR 0x7018U

/**
 * @brief RobStride 电机型号枚举
 */
typedef enum {
    ROBSTRIDE_TYPE_RS00 = 0,  ///< RobStride 00
    ROBSTRIDE_TYPE_RS01,      ///< RobStride O1/RS01，速度 44 rad/s，峰值力矩 17 N*m
    ROBSTRIDE_TYPE_RS05,      ///< RobStride 05，峰值力矩 5.5 N*m
    ROBSTRIDE_TYPE_EL05,      ///< EduLite 05 青春版，峰值力矩 6 N*m
} RobStrideMotorType;

/**
 * @brief RobStride 控制模式枚举
 */
typedef enum {
    ROBSTRIDE_CTRL_STOP = 0,    ///< 停止模式
    ROBSTRIDE_CTRL_NATIVE_MIT,  ///< 原生 MIT 模式
    ROBSTRIDE_CTRL_POS,         ///< 软件位置环模式
    ROBSTRIDE_CTRL_SPD,         ///< 软件速度环模式
} RobStrideControlMode;

/**
 * @brief RobStride 电机反馈状态枚举
 */
typedef enum {
    ROBSTRIDE_STATE_RESET = 0,  ///< 复位态
    ROBSTRIDE_STATE_CALI = 1,   ///< 标定态
    ROBSTRIDE_STATE_RUN = 2,    ///< 运行态
} RobStrideModeState;

/**
 * @brief RobStride 工作模式枚举
 */
typedef enum {
    ROBSTRIDE_WORK_RUN = 0,  ///< 正常运行模式
    ROBSTRIDE_WORK_SCAN,     ///< ID 扫描模式
} RobStrideWorkMode;

/**
 * @brief RobStride 电机限制参数
 */
typedef struct
{
    float p_min;   ///< 最小位置 (rad)
    float p_max;   ///< 最大位置 (rad)
    float v_min;   ///< 最小速度 (rad/s)
    float v_max;   ///< 最大速度 (rad/s)
    float t_min;   ///< 最小力矩 (N*m)
    float t_max;   ///< 最大力矩 (N*m)
    float kp_min;  ///< 最小刚度系数
    float kp_max;  ///< 最大刚度系数
    float kd_min;  ///< 最小阻尼系数
    float kd_max;  ///< 最大阻尼系数
} RobStrideMotorLimit;

/**
 * @brief RobStride 电机 PID 容器
 */
typedef struct
{
    PIDStructTypedef SpeedPID;  ///< 速度环 PID
    PIDStructTypedef PosPID;    ///< 位置环 PID
} RobStrideMotorPID;

/**
 * @brief RobStride 电机运行期数据
 */
typedef struct
{
    float PosRaw;                      ///< 单圈原始位置 (rad)
    RobStrideModeState rx_mode_state;  ///< 反馈模式状态
    uint32_t rx_error_code;            ///< Type 2 故障摘要或 Type 21 完整故障位
    RobStrideMotorLimit limit;         ///< 当前型号限制参数
    RobStrideWorkMode work_mode;       ///< 当前工作模式
    bool scan_ok;                      ///< 本轮扫描是否成功
    uint16_t scan_id;                  ///< 扫描命中的 ID
    uint8_t scan_hit;                  ///< 扫描命中次数
    uint8_t scan_idx;                  ///< 当前扫描目标 ID
    uint8_t scan_start;                ///< 本轮扫描起始 ID
    uint8_t scan_round;                ///< 已完成扫描轮数
} RobStrideMotorData;

/**
 * @brief RobStride 电机状态标志
 */
typedef struct
{
    uint8_t connected : 1;   ///< 反馈在线标志
    uint16_t lost_count;     ///< 连续丢包计数
    uint32_t last_rx_count;  ///< 上次反馈计数
} RobStrideMotorFlag;

/**
 * @brief RobStride 电机对象
 */
typedef struct
{
    uint16_t id;                     ///< 当前目标电机 ID
    uint16_t mst_id;                 ///< 主控 ID
    uint8_t can_bus;                 ///< CAN 总线号
    RobStrideMotorType type;         ///< 电机型号
    RobStrideControlMode ctrl_mode;  ///< 控制模式

    float Pos;  ///< 位置目标 (rad)
    float W;    ///< 速度目标 (rad/s)
    float T;    ///< 力矩目标 (N*m)
    float K_P;  ///< MIT 刚度系数
    float K_W;  ///< MIT 阻尼系数

    float rx_Pos;   ///< 展开后位置反馈 (rad)
    float rx_W;     ///< 速度反馈 (rad/s)
    float rx_T;     ///< 力矩反馈 (N*m)
    float rx_Temp;  ///< 温度反馈 (degC)

    RobStrideMotorFlag flag;  ///< 在线状态标志
    RobStrideMotorPID PID;    ///< 软件 PID 参数
    RobStrideMotorData Data;  ///< 运行期数据

    uint32_t tx_count;  ///< 发送计数
    uint32_t rx_count;  ///< 接收计数
} RobStrideMotor;

/**
 * @brief  初始化 RobStride 电机对象
 * @param  motor: 电机对象指针
 * @param  type:  电机型号
 * @param  can_bus: CAN 总线号 (1 ~ 2)
 * @param  id:    电机 ID (0 ~ 127)
 * @retval 无
 */
void RobStrideMotor_Init(RobStrideMotor* motor, RobStrideMotorType type, uint8_t can_bus, uint32_t id);

/**
 * @brief  更新 RobStride 电机控制量
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Control_Update(RobStrideMotor* motor);

/**
 * @brief  发送 RobStride 电机控制或扫描帧
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Transmit(RobStrideMotor* motor);

/**
 * @brief  使能 RobStride 电机
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Enable(RobStrideMotor* motor);

/**
 * @brief  失能 RobStride 电机
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_Disable(RobStrideMotor* motor);

/**
 * @brief  读取 RobStride 电机 ID
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_GetID(RobStrideMotor* motor);

/**
 * @brief  修改 RobStride 电机 ID
 * @param  bus:    CAN 总线号 (1 ~ 2)
 * @param  old_id: 当前电机 ID (0 ~ 127)
 * @param  new_id: 新电机 ID (0 ~ 127)
 * @retval 无
 */
void RobStrideMotor_SetID(uint8_t bus, uint8_t old_id, uint8_t new_id);

/**
 * @brief  设置当前位置为零点
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_SetZero(RobStrideMotor* motor);

/**
 * @brief  写入 RobStride RAM 参数
 * @param  motor: 电机对象指针
 * @param  index: 参数索引
 * @param  value: 参数值
 * @retval 无
 */
void RobStrideMotor_WriteParam(RobStrideMotor* motor, uint16_t index, float value);

/**
 * @brief  启动 RobStride ID 扫描
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_StartScan(RobStrideMotor* motor);

/**
 * @brief  停止 RobStride ID 扫描
 * @param  motor: 电机对象指针
 * @retval 无
 */
void RobStrideMotor_StopScan(RobStrideMotor* motor);

/**
 * @brief  解析 RobStride 电机接收帧
 * @param  frame: 接收帧指针
 * @param  motor: 电机对象指针
 * @retval 无
 */
void Process_RobStride_Motor_Frame(const FDCANFrame* frame, RobStrideMotor* motor);

/**
 * @brief  通用 FDCAN 分发器适配入口
 * @param  frame: 接收帧指针
 * @param  context: 指向 RobStrideMotor 实例
 * @retval 1 表示该帧已由本电机处理，0 表示不匹配
 */
uint8_t RobStrideMotor_RxHandler(const FDCANFrame* frame, void* context);

#endif
