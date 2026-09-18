/**
 * @file    PID.c
 * @brief   PID controller source file, including period compensation,
 *          feedforward separation and derivative filtering
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-18
 */

#include "PID.h"
#include <math.h>  // 使用 fminf/fmaxf 完成输出限幅

static inline float Pid_Compensate(float val, float compensate);
static inline float Pid_Limit(float val, float limit);
static inline float Pid_PTerm(float error, PIDStructTypedef* pid);
static inline float Pid_ITerm(float error, PIDStructTypedef* pid);
static inline float Pid_DTerm(float error, float present_feedback, PIDStructTypedef* pid);

/**
 * @brief  带周期补偿的 PID 控制器计算函数
 * @param  Reference:        目标值
 * @param  Present_Feedback: 当前反馈值
 * @param  PID_Struct:       PID 控制器结构体指针
 * @retval PID 控制输出值
 */
float Pid_Regulate_Auto(float Reference, float Present_Feedback, PIDStructTypedef* PID_Struct) {
    float BasePeriod = (float)((PID_Struct->flag.BasePeriodMs == 0U) ? PID_DEFAULT_BASE_PERIOD_MS
                                                                     : PID_Struct->flag.BasePeriodMs);
    float ActualPeriod = (float)((PID_Struct->flag.ActualPeriodMs == 0U) ? PID_DEFAULT_ACTUAL_PERIOD_MS
                                                                         : PID_Struct->flag.ActualPeriodMs);

    if (BasePeriod <= 0 || ActualPeriod <= 0 || BasePeriod == ActualPeriod) {
        return Pid_Regulate(Reference, Present_Feedback, PID_Struct);
    }
    // 计算缩放比例
    float ratio = ActualPeriod / BasePeriod;

    // 备份并缩放参数
    float original_Ki = PID_Struct->Ki;
    float original_Kd = PID_Struct->Kd;
    PID_Struct->Ki = original_Ki * ratio;  // 频率越高(ratio越小)，单步积分越小
    PID_Struct->Kd = original_Kd / ratio;  // 频率越高(ratio越小)，微分增益需越大
    float output = Pid_Regulate(Reference, Present_Feedback, PID_Struct);
    PID_Struct->Ki = original_Ki;
    PID_Struct->Kd = original_Kd;
    return output;
}

/**
 * @brief  PID 控制器计算函数
 * @param  Reference:        目标值
 * @param  Present_Feedback: 当前反馈值
 * @param  PID_Struct:       PID 控制器结构体指针
 * @retval PID 控制输出值
 */
float Pid_Regulate(float Reference, float Present_Feedback, PIDStructTypedef* PID_Struct) {
    float Error = Reference - Present_Feedback;
    float Output;

    /* Compute terms */
    float pTerm = Pid_PTerm(Error, PID_Struct);
    float iTerm = Pid_ITerm(Error, PID_Struct);
    float dTerm = Pid_DTerm(Error, Present_Feedback, PID_Struct);

    PID_Struct->PreError = Error;
    PID_Struct->PreMeasure = Present_Feedback;
    PID_Struct->PreDTerm = dTerm;

    Output = pTerm + iTerm + dTerm;

    /*range error check*/
    if (PID_Struct->RangeError < 0)
        PID_Struct->RangeError = 0;
    if (Error < PID_Struct->RangeError && Error > -PID_Struct->RangeError) {
        PID_Struct->Integral = 0;
        Output = 0;
    }

    /* 是否启用前馈分离 */
    if (PID_Struct->flag.Feedforward_Separated) {
        /* 对反馈/基本控制输出部分进行补偿和限幅 */
        Output = Pid_Compensate(Output, PID_Struct->Compensate);
        Output = Pid_Limit(Output, PID_Struct->LimitOutput);

        Output += Pid_Limit(PID_Struct->Feedforward, PID_Struct->LimitOutput);
    } else {
        /* 未分离时，前馈直接叠加到基本输出，然后整体进行补偿和限幅 */
        Output += PID_Struct->Feedforward;
        Output = Pid_Compensate(Output, PID_Struct->Compensate);
        Output = Pid_Limit(Output, PID_Struct->LimitOutput);
    }

    return Output;
}

/**
 * @brief  输出补偿处理
 * @param  val:        待补偿输出值
 * @param  compensate: 补偿量
 * @retval 补偿后的输出值
 */
static inline float Pid_Compensate(float val, float compensate) {
    if (val > 0.0f) {
        val += compensate;
        if (val < 0.0f)
            val = 0.0f;
    } else if (val < 0.0f) {
        val -= compensate;
        if (val > 0.0f)
            val = 0.0f;
    }
    return val;
}

/**
 * @brief  输出限幅处理
 * @param  val:   待限幅值
 * @param  limit: 正负对称限幅值
 * @retval 限幅后的输出值
 */
static inline float Pid_Limit(float val, float limit) {
    if (limit < 0.0f)
        limit = 0.0f;
    return fmaxf(-limit, fminf(val, limit));
}

/**
 * @brief  计算比例项
 * @param  error: 当前误差
 * @param  pid:   PID 控制器结构体指针
 * @retval 比例项输出
 */
static inline float Pid_PTerm(float error, PIDStructTypedef* pid) {
    return error * pid->Kp;
}

/**
 * @brief  计算积分项并执行积分限幅
 * @param  error: 当前误差
 * @param  pid:   PID 控制器结构体指针
 * @retval 积分项输出
 */
static inline float Pid_ITerm(float error, PIDStructTypedef* pid) {
    if (pid->Ki == 0.0f) {
        pid->Integral = 0.0f;
    } else {
        float iTerm = error * pid->Ki;
        float new_integral = pid->Integral + iTerm;

        /*limit integral*/
        if (new_integral > pid->LimitIntegral)
            pid->Integral = pid->LimitIntegral;
        else if (new_integral < -pid->LimitIntegral)
            pid->Integral = -pid->LimitIntegral;
        else
            pid->Integral = new_integral;
    }
    return pid->Integral;
}

/**
 * @brief  计算微分项
 * @param  error:            当前误差
 * @param  present_feedback: 当前反馈值
 * @param  pid:              PID 控制器结构体指针
 * @retval 微分项输出
 */
static inline float Pid_DTerm(float error, float present_feedback, PIDStructTypedef* pid) {
    float dTerm = 0.0f;
    if (pid->flag.Derivative_On_Measurement) {
        // 微分先行：只对测量值进行微分 (Error = Target - Measure)
        float Measure_Inc = present_feedback - pid->PreMeasure;
        dTerm = -Measure_Inc * pid->Kd;
    } else {
        // 传统微分：对误差进行微分
        float Error_Inc = error - pid->PreError;
        dTerm = Error_Inc * pid->Kd;
    }

    // 微分项一阶低通滤波 (PT1)
    if (pid->D_Filter_Alpha > 0.0f && pid->D_Filter_Alpha <= 1.0f) {
        dTerm = pid->D_Filter_Alpha * dTerm + (1.0f - pid->D_Filter_Alpha) * pid->PreDTerm;
    }

    return dTerm;
}
