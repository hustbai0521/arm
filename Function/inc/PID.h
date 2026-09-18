/**
 * @file    PID.h
 * @brief   PID controller header file, defining parameters, states and APIs
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-18
 */

#ifndef _PID_H_
#define _PID_H_

#include "cmsis_os2.h"
#include <stdbool.h>

#define PID_DEFAULT_BASE_PERIOD_MS (10U)
#define PID_DEFAULT_ACTUAL_PERIOD_MS (10U)

/**
 * @brief PID 控制器参数与运行状态结构体
 */
typedef struct
{
    float Kp;              ///< 比例系数
    float Ki;              ///< 积分系数
    float Kd;              ///< 微分系数
    float LimitOutput;     ///< 输出限幅值，按正负对称限幅处理
    float LimitIntegral;   ///< 积分限幅值，按正负对称限幅处理
    float Integral;        ///< 积分项累加值，由 Pid_ITerm 更新
    float Compensate;      ///< 输出静摩擦/死区补偿值，按输出符号叠加
    float Feedforward;     ///< 前馈项，可选择参与整体限幅或独立叠加
    float RangeError;      ///< 误差死区范围，误差进入死区后清零积分和输出
    float PreError;        ///< 上一次误差，用于传统误差微分
    float PreMeasure;      ///< 上一次测量值，用于微分先行
    float PreDTerm;        ///< 上一次微分项，用于 D 项低通滤波
    float D_Filter_Alpha;  ///< D 项一阶低通滤波系数 (0.0~1.0, 1.0: 不滤波)
    struct
    {
        bool Feedforward_Separated;      ///< 前馈分离标志 (true: 前馈量在 PID 输出限幅后独立叠加)
        bool Derivative_On_Measurement;  ///< 微分先行标志 (true: 对测量值微分)
        uint16_t BasePeriodMs;           ///< 调参时使用的基础控制周期 (单位: ms)
        uint16_t ActualPeriodMs;         ///< 当前实际运行控制周期 (单位: ms)
    } flag;
} PIDStructTypedef;

/**
 * @brief  PID 控制器计算函数
 * @param  Reference:        目标值
 * @param  Present_Feedback: 当前反馈值
 * @param  PID_Struct:       PID 控制器结构体指针
 * @retval PID 控制输出值
 */
float Pid_Regulate(float Reference, float Present_Feedback, PIDStructTypedef* PID_Struct);

/**
 * @brief  带周期补偿的 PID 控制器计算函数
 * @param  Reference:        目标值
 * @param  Present_Feedback: 当前反馈值
 * @param  PID_Struct:       PID 控制器结构体指针
 * @retval PID 控制输出值
 */
float Pid_Regulate_Auto(float Reference, float Present_Feedback, PIDStructTypedef* PID_Struct);

#endif /* _PID_H_ */
