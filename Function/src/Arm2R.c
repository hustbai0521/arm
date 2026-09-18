/**
 * @file    Arm2R.c
 * @brief   Planar 2R arm kinematics and trajectory planning source file
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-18
 */
#include "Arm2R.h"
#include "FreeRTOS.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/// 启用快速数学库；注释此宏可回退到标准数学库
// #define USE_MATH_LIB

// #ifdef USE_MATH_LIB
// #include "HUST_Math_Lib.h"
// #endif

/* ==================== 曲线路径辅助结构与函数 ==================== */

/**
 * @brief 直线路径私有数据结构体
 */
typedef struct
{
    float start_x;  ///< 起点 X 坐标
    float start_y;  ///< 起点 Y 坐标
    float end_x;    ///< 终点 X 坐标
    float end_y;    ///< 终点 Y 坐标
    float dir_x;    ///< 单位切向量 X 分量
    float dir_y;    ///< 单位切向量 Y 分量
} LinearPathData;

static void Arm2R_JointToXY(float b0, float j0, float L1, float L2, float rad_to_base, float rad_to_joint,
                            float joint_off, float* x0, float* y0);

/**
 * @brief  采样直线路径点
 * @param  ctx:       路径上下文指针
 * @param  u:         路径归一化进度 (范围: 0.0~1.0)
 * @param  out_x:     输出 X 坐标指针
 * @param  out_y:     输出 Y 坐标指针
 * @param  out_dir_x: 输出切向量 X 分量指针
 * @param  out_dir_y: 输出切向量 Y 分量指针
 * @retval true: 采样成功
 */
static bool _LinearPath(Arm2R_PathContext* ctx, float u, float* out_x, float* out_y, float* out_dir_x, float* out_dir_y) {
    LinearPathData* data = (LinearPathData*)ctx->data;
    *out_x = data->start_x + u * (data->end_x - data->start_x);
    *out_y = data->start_y + u * (data->end_y - data->start_y);
    *out_dir_x = data->dir_x;
    *out_dir_y = data->dir_y;
    return true;
}

/**
 * @brief 二阶贝塞尔路径私有数据结构体
 */
typedef struct
{
    float p0_x, p0_y;  ///< 起点坐标
    float p1_x, p1_y;  ///< 控制点坐标
    float p2_x, p2_y;  ///< 终点坐标
} Bezier2PathData;

/**
 * @brief  采样二阶贝塞尔路径点
 * @param  ctx:       路径上下文指针
 * @param  u:         路径归一化进度 (范围: 0.0~1.0)
 * @param  out_x:     输出 X 坐标指针
 * @param  out_y:     输出 Y 坐标指针
 * @param  out_dir_x: 输出切向量 X 分量指针
 * @param  out_dir_y: 输出切向量 Y 分量指针
 * @retval true: 采样成功
 */
static bool _Bezier2Path(Arm2R_PathContext* ctx, float u, float* out_x, float* out_y, float* out_dir_x,
                         float* out_dir_y) {
    Bezier2PathData* data = (Bezier2PathData*)ctx->data;

    float inv_u = 1.0f - u;
    float inv_u2 = inv_u * inv_u;
    float u2 = u * u;

    // 位置方程： P(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
    *out_x = inv_u2 * data->p0_x + 2.0f * inv_u * u * data->p1_x + u2 * data->p2_x;
    *out_y = inv_u2 * data->p0_y + 2.0f * inv_u * u * data->p1_y + u2 * data->p2_y;

    // 导数方程（切向量）： P'(t) = 2(1-t)(P1-P0) + 2t(P2-P1)
    float dx = 2.0f * inv_u * (data->p1_x - data->p0_x) + 2.0f * u * (data->p2_x - data->p1_x);
    float dy = 2.0f * inv_u * (data->p1_y - data->p0_y) + 2.0f * u * (data->p2_y - data->p1_y);

    float len = sqrtf(dx * dx + dy * dy);
    if (len > 1e-6f) {
        *out_dir_x = dx / len;
        *out_dir_y = dy / len;
    } else {
        *out_dir_x = 0.0f;
        *out_dir_y = 0.0f;
    }

    return true;
}

/**
 * @brief  估算二阶贝塞尔曲线长度
 * @param  data:  二阶贝塞尔路径数据指针
 * @param  steps: 离散采样步数
 * @retval 曲线近似长度
 */
static float _Bezier2Length(Bezier2PathData* data, int steps) {
    float len = 0.0f;
    float prev_x = data->p0_x;
    float prev_y = data->p0_y;

    // 构造临时上下文以复用求值函数
    Arm2R_PathContext temp_ctx = {.data = data};

    // 使用步长积分配
    for (int i = 1; i <= steps; i++) {
        float u = (float)i / steps;
        float cx, cy, dir_x, dir_y;

        // 直接调用评估函数获取当前点坐
        _Bezier2Path(&temp_ctx, u, &cx, &cy, &dir_x, &dir_y);

        float dx = cx - prev_x;
        float dy = cy - prev_y;
        len += sqrtf(dx * dx + dy * dy);

        prev_x = cx;
        prev_y = cy;
    }
    return len;
}

/**
 * @brief 三阶贝塞尔路径私有数据结构体
 */
typedef struct
{
    float p0_x, p0_y;  ///< 起点坐标
    float p1_x, p1_y;  ///< 第一控制点坐标
    float p2_x, p2_y;  ///< 第二控制点坐标
    float p3_x, p3_y;  ///< 终点坐标
} Bezier3PathData;

/**
 * @brief  采样三阶贝塞尔路径点
 * @param  ctx:       路径上下文指针
 * @param  u:         路径归一化进度 (范围: 0.0~1.0)
 * @param  out_x:     输出 X 坐标指针
 * @param  out_y:     输出 Y 坐标指针
 * @param  out_dir_x: 输出切向量 X 分量指针
 * @param  out_dir_y: 输出切向量 Y 分量指针
 * @retval true: 采样成功
 */
static bool _Bezier3Path(Arm2R_PathContext* ctx, float u, float* out_x, float* out_y, float* out_dir_x,
                         float* out_dir_y) {
    Bezier3PathData* data = (Bezier3PathData*)ctx->data;

    float inv_u = 1.0f - u;
    float inv_u2 = inv_u * inv_u;
    float inv_u3 = inv_u2 * inv_u;
    float u2 = u * u;
    float u3 = u2 * u;

    // 位置方程： P(t) = (1-t)^3 * P0 + 3(1-t)^2 * t * P1 + 3(1-t) * t^2 * P2 + t^3 * P3
    *out_x = inv_u3 * data->p0_x + 3.0f * inv_u2 * u * data->p1_x + 3.0f * inv_u * u2 * data->p2_x + u3 * data->p3_x;
    *out_y = inv_u3 * data->p0_y + 3.0f * inv_u2 * u * data->p1_y + 3.0f * inv_u * u2 * data->p2_y + u3 * data->p3_y;

    // 导数方程（切向量）： P'(t) = 3(1-t)^2(P1-P0) + 6(1-t)t(P2-P1) + 3t^2(P3-P2)
    float dx = 3.0f * inv_u2 * (data->p1_x - data->p0_x) + 6.0f * inv_u * u * (data->p2_x - data->p1_x) +
               3.0f * u2 * (data->p3_x - data->p2_x);
    float dy = 3.0f * inv_u2 * (data->p1_y - data->p0_y) + 6.0f * inv_u * u * (data->p2_y - data->p1_y) +
               3.0f * u2 * (data->p3_y - data->p2_y);

    float len = sqrtf(dx * dx + dy * dy);
    if (len > 1e-6f) {
        *out_dir_x = dx / len;
        *out_dir_y = dy / len;
    } else {
        *out_dir_x = 0.0f;
        *out_dir_y = 0.0f;
    }

    return true;
}

/**
 * @brief  估算三阶贝塞尔曲线长度
 * @param  data:  三阶贝塞尔路径数据指针
 * @param  steps: 离散采样步数
 * @retval 曲线近似长度
 */
static float _Bezier3Length(Bezier3PathData* data, int steps) {
    float len = 0.0f;
    float prev_x = data->p0_x;
    float prev_y = data->p0_y;

    // 构造临时上下文以复用求值函数
    Arm2R_PathContext temp_ctx = {.data = data};

    // 使用步长积分配
    for (int i = 1; i <= steps; i++) {
        float u = (float)i / steps;
        float cx, cy, dir_x, dir_y;

        _Bezier3Path(&temp_ctx, u, &cx, &cy, &dir_x, &dir_y);

        float dx = cx - prev_x;
        float dy = cy - prev_y;
        len += sqrtf(dx * dx + dy * dy);

        prev_x = cx;
        prev_y = cy;
    }
    return len;
}

/* ==================== 通用 Helper 函数 ==================== */

/**
 * @brief 将接近关节边界的数值吸附到合法边界，避免微小误差触发错误分支
 * @param value: 待处理的关节值
 * @retval 吸附后的关节值
 */
static float Arm2R_SnapJoint(float value) {
    if (value > -ARM2R_IK_TOLERANCE && value < 0.0f)
        return 0.0f;
    if (value < 100.0f + ARM2R_IK_TOLERANCE && value > 100.0f)
        return 100.0f;
    return value;
}

/**
 * @brief 将 Base 关节解按周期折回合法软限位附近
 * @param value:  待处理的 Base 关节值
 * @param period: Base 关节一整圈对应的 unit 数
 * @retval 折回并吸附后的 Base 关节值；若无法折回合法区间则返回原值
 */
static float Arm2R_WrapBase(float value, float period) {
    float normalized;

    if (period <= ARM_EPSILON)
        return value;

    normalized = value;
    while (normalized < -ARM2R_IK_TOLERANCE)
        normalized += period;
    if (normalized <= 100.0f + ARM2R_IK_TOLERANCE)
        return Arm2R_SnapJoint(normalized);

    normalized = value;
    while (normalized > 100.0f + ARM2R_IK_TOLERANCE)
        normalized -= period;
    if (normalized >= -ARM2R_IK_TOLERANCE)
        return Arm2R_SnapJoint(normalized);

    return value;
}

/**
 * @brief 由当前关节值计算笛卡尔起点
 * @param b0:           当前 Base 关节位置 (unit)
 * @param j0:           当前 Joint 关节位置 (unit)
 * @param L1:           大臂长度
 * @param L2:           小臂长度
 * @param rad_to_base:  弧度转 Base 关节 unit 系数
 * @param rad_to_joint: 弧度转 Joint 关节 unit 系数
 * @param joint_off:    Joint 关节安装偏置 (unit)
 * @param x0:           输出起点 X 坐标指针
 * @param y0:           输出起点 Y 坐标指针
 */
static void Arm2R_JointToXY(float b0, float j0, float L1, float L2, float rad_to_base, float rad_to_joint,
                            float joint_off, float* x0, float* y0) {
    const float rad_to_deg = 180.0f / 3.14159265358979323846f;
    Arm2R_FK(b0, j0, L1, L2, rad_to_deg / rad_to_base, rad_to_deg / rad_to_joint,
             joint_off * (rad_to_deg / rad_to_joint), 3.14159265358979323846f / 180.0f, x0, y0);
}

/* ==================== Arm2R 正式库函数 ==================== */

/**
 * @brief  计算平面二自由度机械臂正运动学
 * @param  b:            Base 关节位置 (单位: unit)
 * @param  j:            Joint 关节位置 (单位: unit)
 * @param  L1:           大臂长度
 * @param  L2:           小臂长度
 * @param  base_to_deg:  Base 关节 unit 转角度系数
 * @param  joint_to_deg: Joint 关节 unit 转角度系数
 * @param  joint_off:    Joint 关节安装偏置角 (单位: deg)
 * @param  deg_to_rad:   角度转弧度系数
 * @param  x:            输出末端 X 坐标指针
 * @param  y:            输出末端 Y 坐标指针
 * @retval 无
 */
inline void Arm2R_FK(float b, float j, float L1, float L2, float base_to_deg, float joint_to_deg, float joint_off,
                     float deg_to_rad, float* x, float* y) {
    float th1 = b * base_to_deg * deg_to_rad;
    float th2 = (joint_off - j * joint_to_deg) * deg_to_rad;

    *x = L1 * cosf(th1) + L2 * cosf(th1 + th2);
    *y = L1 * sinf(th1) + L2 * sinf(th1 + th2);
}

/**
 * @brief  生成速度规划曲线
 * @param  dist:           路径总距离
 * @param  vmax:           最大速度
 * @param  amax:           最大加速度
 * @param  vel_curve:      输出速度曲线数组
 * @param  max_points:     速度曲线数组容量
 * @param  out_total_time: 输出规划总时间指针
 * @retval true: 成功, false: 参数无效或规划失败
 */
bool Arm2R_SpeedPlan(float dist, float vmax, float amax, float* vel_curve, int max_points, float* out_total_time) {
    if (dist <= 0.001f || vmax <= 0.001f || max_points < 2)
        return false;

    // --- 参数定义 ---
    float v_min = ARM2R_SPEED_PLAN_MIN_FACTOR;
    float cs, t_accel, t_decel, t_cruise;

    // 1. 计算达到的最高速度 cs 和减速所需时间
    if (amax > 0.001f) {
        // 考虑 v_min 的情况下，判断距离是否足够加速到 vmax
#if ARM2R_SPEED_PLAN_INCLUDE_ACCEL
        // 打开该宏后，速度规划同时考虑加速段和减速段
        // 因此最高速度上限需要同时预留两侧的速度变化距离
        float vmax_limit = sqrtf((dist * amax * ARM2R_SPEED_PLAN_ACCEL_FACTOR) / 1.5f + (v_min * v_min));
#else
        // 关闭该宏时，保持原行为：只显式考虑减速段
        float vmax_limit = sqrtf(2.0f * dist * amax + (v_min * v_min));
#endif
        cs = fminf(vmax, vmax_limit);

#if ARM2R_SPEED_PLAN_INCLUDE_ACCEL
        // 加速/减速时间：从 v_min 到 cs，再从 cs 回到 v_min
        // 注意：这里仍保留原来的 1.5 倍系数，维持速度变化的平滑性
        t_accel = (cs - v_min) / (amax * ARM2R_SPEED_PLAN_ACCEL_FACTOR);
        t_decel = t_accel;
#else
        // 只保留减速段时，不使用加速段系数，保持 vmax_limit 与减速距离一致
        t_accel = 0.0f;
        t_decel = (cs - v_min) / amax;
#endif
    } else {
        cs = vmax;
        t_accel = 0.0f;
        t_decel = 0.0f;
    }

    // 2. 计算距离和时间分配
    // 加速/减速段平均速度 V_avg = (cs + v_min) / 2
    float accel_dist = 0.5f * (cs + v_min) * t_accel;
    float decel_dist = 0.5f * (cs + v_min) * t_decel;

#if ARM2R_SPEED_PLAN_INCLUDE_ACCEL
    // 安全检查：如果加速+减速距离超过了总距离，则取消巡航段并等比压缩两侧时间
    if (accel_dist + decel_dist > dist) {
        t_accel = dist / (cs + v_min);
        t_decel = t_accel;
        t_cruise = 0.0f;
    } else {
        float cruise_dist = dist - accel_dist - decel_dist;
        t_cruise = cruise_dist / cs;
    }

    float total_time = t_accel + t_cruise + t_decel;
#else
    // 安全检查：如果距离太短，减速距离超过了总距离
    if (decel_dist > dist) {
        decel_dist = dist;
        t_decel = decel_dist / (0.5f * (cs + v_min));
        t_cruise = 0.0f;
    } else {
        float cruise_dist = dist - decel_dist;
        t_cruise = cruise_dist / cs;
    }

    float total_time = t_cruise + t_decel;
#endif
    *out_total_time = total_time;

    // 3. 计算采样步长
    float dt = total_time / (max_points - 1);

    // 4. 填充数组
    for (int i = 0; i < max_points; i++) {
        float t = i * dt;
        float vel = 0.0f;

#if ARM2R_SPEED_PLAN_INCLUDE_ACCEL
        if (t <= t_accel) {
            // 加速段：从 v_min 平滑爬升到 cs
            float progress = t / (t_accel + 1e-9f);
            progress = fmaxf(0.0f, fminf(1.0f, progress));
            float smooth = progress * progress * (3.0f - 2.0f * progress);
            vel = v_min + (cs - v_min) * smooth;
        } else if (t <= t_accel + t_cruise) {
            vel = cs;
        } else {
            // 减速段：t 落在 [t_accel + t_cruise, total_time]
            float progress = (t - t_accel - t_cruise) / (t_decel + 1e-9f);
            progress = fmaxf(0.0f, fminf(1.0f, progress));

            // S-Curve 映射
            float smooth = progress * progress * (3.0f - 2.0f * progress);

            // 从 cs 平滑降到 v_min
            vel = cs - (cs - v_min) * smooth;
        }
#else
        if (t <= t_cruise) {
            vel = cs;
        } else {
            // 减速段：t 落在 [t_cruise, total_time]
            float progress = (t - t_cruise) / (t_decel + 1e-9f);
            progress = fmaxf(0.0f, fminf(1.0f, progress));

            // S-Curve 映射
            float smooth = progress * progress * (3.0f - 2.0f * progress);

            // 关键修改：从 cs 降到 0
            vel = cs - (cs - v_min) * smooth;
        }
#endif
        vel_curve[i] = vel;
    }

    // 5. 强制终点：物理上的最后一步必须为 0，以实现静止
    vel_curve[max_points - 1] = 0.0f;

    return true;
}

/**
 * @brief  计算平面二自由度机械臂逆运动学
 * @param  x:            目标 X 坐标
 * @param  y:            目标 Y 坐标
 * @param  L1:           大臂长度
 * @param  L2:           小臂长度
 * @param  rad_to_base:  弧度转 Base 关节 unit 系数
 * @param  rad_to_joint: 弧度转 Joint 关节 unit 系数
 * @param  joint_off:    Joint 关节安装偏置
 * @param  clamp_radius: 是否将目标半径钳制到可达工作空间
 * @param  b1:           输出第一组 Base 逆解指针
 * @param  j1:           输出第一组 Joint 逆解指针
 * @param  b2:           输出第二组 Base 逆解指针
 * @param  j2:           输出第二组 Joint 逆解指针
 * @param  v1:           输出第一组逆解有效标志指针
 * @param  v2:           输出第二组逆解有效标志指针
 * @retval true: 至少存在一组有效解, false: 无有效解
 */
inline bool Arm2R_IK(float x, float y, float L1, float L2, float rad_to_base, float rad_to_joint, float joint_off,
                     bool clamp_radius, float* b1, float* j1, float* b2, float* j2, bool* v1, bool* v2) {
    float L1_sq = L1 * L1;
    float L2_sq = L2 * L2;
    float two_L1_L2 = 2.0f * L1 * L2;
    float r2 = x * x + y * y;
    float min_reach = fabsf(L1 - L2);
    float max_reach = L1 + L2;
    float d = sqrtf(r2);

    if (!clamp_radius && (d < min_reach - 1e-6f || d > max_reach + 1e-6f)) {
        *b1 = 0.0f;
        *j1 = 0.0f;
        *b2 = 0.0f;
        *j2 = 0.0f;
        *v1 = false;
        *v2 = false;
        return false;
    }

    if (clamp_radius) {
        float d_clamp = fmaxf(min_reach, fminf(max_reach, d));
        r2 = d_clamp * d_clamp;
    }

    {
        float cos_t2 = fmaxf(-1.0f, fminf(1.0f, (r2 - L1_sq - L2_sq) / two_L1_L2));
        float sin_t2 = sqrtf(fmaxf(0.0f, 1.0f - cos_t2 * cos_t2));

#ifdef USE_MATH_LIB
        float alpha = FastTableAtan2(y, x);
        float beta = FastTableAtan2(L2 * sin_t2, L1 + L2 * cos_t2);
        float t2_abs = FastAcos(cos_t2);
#else
        float alpha = atan2f(y, x);
        float beta = atan2f(L2 * sin_t2, L1 + L2 * cos_t2);
        float t2_abs = acosf(cos_t2);
#endif

        *b1 = (alpha - beta) * rad_to_base;
        *j1 = joint_off - t2_abs * rad_to_joint;
        *b2 = (alpha + beta) * rad_to_base;
        *j2 = joint_off + t2_abs * rad_to_joint;

        {
            float base_period = 2.0f * 3.14159265358979323846f * rad_to_base;
            *b1 = Arm2R_WrapBase(*b1, base_period);
            *b2 = Arm2R_WrapBase(*b2, base_period);
        }
        *j1 = Arm2R_SnapJoint(*j1);
        *j2 = Arm2R_SnapJoint(*j2);
    }

    *v1 = (*b1 >= -ARM2R_IK_TOLERANCE && *b1 <= 100.0f + ARM2R_IK_TOLERANCE &&
           *j1 >= -ARM2R_IK_TOLERANCE && *j1 <= 100.0f + ARM2R_IK_TOLERANCE);
    *v2 = (*b2 >= -ARM2R_IK_TOLERANCE && *b2 <= 100.0f + ARM2R_IK_TOLERANCE &&
           *j2 >= -ARM2R_IK_TOLERANCE && *j2 <= 100.0f + ARM2R_IK_TOLERANCE);
    return (*v1 || *v2);
}

/**
 * @brief  根据关节空间路径上下文构建关节轨迹
 * @param  path:          关节空间路径上下文指针
 * @param  vmax:          路径最大速度
 * @param  amax:          路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildJointTrajectory(Arm2R_PathContext* path, float vmax, float amax, int dt_ms, int max_points,
                                   float* vel_curve_buf, float* out_b, float* out_j, float* out_vb, float* out_vj,
                                   int* out_count, float* out_time) {
    uint8_t ret = 0;
    float dist = path->length;
    float total_time = 0.0f;
    int plan_points = 0;
    do {
        if (dist < 1e-6f || max_points < 2 || vel_curve_buf == NULL) {
            *out_count = 1;
            if (out_time)
                *out_time = 0.0f;
            out_b[0] = ((LinearPathData*)path->data)->start_x;
            out_j[0] = ((LinearPathData*)path->data)->start_y;
            out_vb[0] = 0;
            out_vj[0] = 0;
            break;
        }

        if (!Arm2R_SpeedPlan(dist, vmax, amax, vel_curve_buf, max_points, &total_time)) {
            ret = 2;
            break;
        }
        if (total_time <= 1e-6f) {
            ret = 2;
            break;
        }

        int origin_dt_ms = dt_ms;
        float min_dt_sec = total_time / (max_points - 1);
        while (dt_ms * 0.001f < min_dt_sec) {
            dt_ms += origin_dt_ms;
        }

        plan_points = (int)ceilf(total_time / (dt_ms * 0.001f)) + 1;
        if (plan_points < 2)
            plan_points = 2;
        if (plan_points > max_points)
            plan_points = max_points;

        if (!Arm2R_SpeedPlan(dist, vmax, amax, vel_curve_buf, plan_points, &total_time)) {
            ret = 2;
            break;
        }
        if (total_time <= 1e-6f) {
            ret = 2;
            break;
        }

        float sample_dt_sec = total_time / (plan_points - 1);
        float current_s = 0.0f;

        for (int i = 0; i < plan_points; i++) {
            float u = (i == plan_points - 1) ? 1.0f : (current_s / dist);
            if (u > 1.0f)
                u = 1.0f;

            float cx, cy, dir_x, dir_y;
            path->evaluate(path, u, &cx, &cy, &dir_x, &dir_y);

            out_b[i] = cx;
            out_j[i] = cy;

            if (i == plan_points - 1 || u >= 1.0f) {
                out_vb[i] = 0.0f;
                out_vj[i] = 0.0f;
                *out_count = i + 1;
                plan_points = i + 1;
                break;
            }

            out_vb[i] = dir_x * vel_curve_buf[i];
            out_vj[i] = dir_y * vel_curve_buf[i];

            if (out_b[i] < 0.0f - ARM2R_IK_TOLERANCE || out_b[i] > 100.0f + ARM2R_IK_TOLERANCE ||
                out_j[i] < 0.0f - ARM2R_IK_TOLERANCE || out_j[i] > 100.0f + ARM2R_IK_TOLERANCE) {
                ret = 4;
                break;
            }

            current_s += vel_curve_buf[i] * sample_dt_sec;
        }

    } while (0);

    if (ret == 0) {
        *out_count = plan_points;
        if (out_time)
            *out_time = total_time;
    }
    return ret;  // SUCCESS
}

/**
 * @brief  根据笛卡尔路径上下文构建关节轨迹
 * @param  path:          路径上下文指针
 * @param  b0:            当前 Base 关节位置 (单位: unit)
 * @param  j0:            当前 Joint 关节位置 (单位: unit)
 * @param  vmax:          路径最大速度
 * @param  amax:          路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  L1:            大臂长度
 * @param  L2:            小臂长度
 * @param  rad_to_base:   弧度转 Base 关节 unit 系数
 * @param  rad_to_joint:  弧度转 Joint 关节 unit 系数
 * @param  joint_off:     Joint 关节安装偏置
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildCartesianTrajectory(Arm2R_PathContext* path, float b0, float j0, float vmax, float amax, int dt_ms,
                                       int max_points, float* vel_curve_buf, float L1, float L2, float rad_to_base,
                                       float rad_to_joint, float joint_off, float* out_b, float* out_j, float* out_vb,
                                       float* out_vj, int* out_count, float* out_time) {
    uint8_t ret = 0;
    float dist = path->length;
    float total_time = 0.0f;
    float sample_dt_sec = 0.0f;
    int plan_points = 0;
    int selected_group = 0;
    do {
        if (dist < 1e-6f || max_points < 2 || vel_curve_buf == NULL) {
            *out_count = 1;
            if (out_time)
                *out_time = 0.0f;
            out_b[0] = b0;
            out_j[0] = j0;
            out_vb[0] = 0;
            out_vj[0] = 0;
            break;
        }

        if (!Arm2R_SpeedPlan(dist, vmax, amax, vel_curve_buf, max_points, &total_time)) {
            ret = 2;
            break;
        }
        if (total_time <= 1e-6f) {
            ret = 2;
            break;
        }

        int origin_dt_ms = dt_ms;
        float min_dt_sec = total_time / (max_points - 1);
        while (dt_ms * 0.001f < min_dt_sec) {
            dt_ms += origin_dt_ms;
        }

        plan_points = (int)ceilf(total_time / (dt_ms * 0.001f)) + 1;
        if (plan_points < 2)
            plan_points = 2;
        if (plan_points > max_points)
            plan_points = max_points;

        if (!Arm2R_SpeedPlan(dist, vmax, amax, vel_curve_buf, plan_points, &total_time)) {
            ret = 2;
            break;
        }
        if (total_time <= 1e-6f) {
            ret = 2;
            break;
        }

        float xi0, yi0, dir_x0, dir_y0;
        path->evaluate(path, 0.0f, &xi0, &yi0, &dir_x0, &dir_y0);

        float b1_ik0, j1_ik0, b2_ik0, j2_ik0;
        bool v1_0, v2_0;
        Arm2R_IK(xi0, yi0, L1, L2, rad_to_base, rad_to_joint, joint_off, false, &b1_ik0, &j1_ik0, &b2_ik0, &j2_ik0,
                 &v1_0, &v2_0);

        if (v1_0 && v2_0) {
            float d1 = (b1_ik0 - b0) * (b1_ik0 - b0) + (j1_ik0 - j0) * (j1_ik0 - j0);
            float d2 = (b2_ik0 - b0) * (b2_ik0 - b0) + (j2_ik0 - j0) * (j2_ik0 - j0);
            selected_group = (d1 <= d2) ? 1 : 2;
        } else if (v1_0) {
            selected_group = 1;
        } else if (v2_0) {
            selected_group = 2;
        } else {
            ret = 4;
            break;
        }

        sample_dt_sec = total_time / (plan_points - 1);
        float current_s = 0.0f;
        float prev_b = b0;
        float prev_j = j0;

        for (int i = 0; i < plan_points; i++) {
            float u = (i == plan_points - 1) ? 1.0f : (current_s / dist);
            if (u > 1.0f)
                u = 1.0f;

            float xi, yi, dir_tx, dir_ty;
            path->evaluate(path, u, &xi, &yi, &dir_tx, &dir_ty);

            float b1_ik, j1_ik, b2_ik, j2_ik;
            bool v1, v2;
            Arm2R_IK(xi, yi, L1, L2, rad_to_base, rad_to_joint, joint_off, false, &b1_ik, &j1_ik, &b2_ik, &j2_ik, &v1,
                     &v2);

            float bi, ji;
            bool v_ik;
            if (selected_group == 1) {
                bi = b1_ik;
                ji = j1_ik;
                v_ik = v1;
            } else {
                bi = b2_ik;
                ji = j2_ik;
                v_ik = v2;
            }

            if (!v_ik) {
                ret = 4;
                break;
            }

            out_b[i] = bi;
            out_j[i] = ji;

            if (i > 0) {
                out_vb[i - 1] = (bi - prev_b) / sample_dt_sec;
                out_vj[i - 1] = (ji - prev_j) / sample_dt_sec;
            }

            prev_b = bi;
            prev_j = ji;

            if (i == plan_points - 1 || u >= 1.0f) {
                out_vb[i] = 0.0f;
                out_vj[i] = 0.0f;
                *out_count = i + 1;
                plan_points = i + 1;
                break;
            }

            if (out_b[i] < 0.0f - ARM2R_IK_TOLERANCE || out_b[i] > 100.0f + ARM2R_IK_TOLERANCE ||
                out_j[i] < 0.0f - ARM2R_IK_TOLERANCE || out_j[i] > 100.0f + ARM2R_IK_TOLERANCE) {
                ret = 4;
                break;
            }

            current_s += vel_curve_buf[i] * sample_dt_sec;
        }

    } while (0);
    if (ret == 0) {
        if (plan_points > 1) {
            float xi_end, yi_end, dir_x_end, dir_y_end;
            path->evaluate(path, 1.0f, &xi_end, &yi_end, &dir_x_end, &dir_y_end);

            float b1_ik, j1_ik, b2_ik, j2_ik;
            bool v1, v2;
            Arm2R_IK(xi_end, yi_end, L1, L2, rad_to_base, rad_to_joint, joint_off, false, &b1_ik, &j1_ik, &b2_ik,
                     &j2_ik, &v1, &v2);

            {
                int last = plan_points - 1;
                float final_b;
                float final_j;
                bool final_valid;

                if (selected_group == 1) {
                    final_b = b1_ik;
                    final_j = j1_ik;
                    final_valid = v1;
                } else {
                    final_b = b2_ik;
                    final_j = j2_ik;
                    final_valid = v2;
                }

                if (!final_valid) {
                    ret = 4;
                } else {
                    out_b[last] = final_b;
                    out_j[last] = final_j;
                    out_vb[last - 1] = (final_b - out_b[last - 1]) / sample_dt_sec;
                    out_vj[last - 1] = (final_j - out_j[last - 1]) / sample_dt_sec;
                    out_vb[last] = 0.0f;
                    out_vj[last] = 0.0f;
                }
            }
        } else if (plan_points == 1) {
            out_b[0] = b0;
            out_j[0] = j0;
            out_vb[0] = 0.0f;
            out_vj[0] = 0.0f;
        }
        if (ret == 0) {
            *out_count = plan_points;
            if (out_time)
                *out_time = total_time;
        }
    }
    return ret;  // SUCCESS
}

/**
 * @brief  构建关节空间直线轨迹
 * @param  b0:            起点 Base 关节位置 (单位: unit)
 * @param  j0:            起点 Joint 关节位置 (单位: unit)
 * @param  b1:            终点 Base 关节位置 (单位: unit)
 * @param  j1:            终点 Joint 关节位置 (单位: unit)
 * @param  vmax:          关节空间路径最大速度
 * @param  amax:          关节空间路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildJoint(float b0, float j0, float b1, float j1, float vmax, float amax, int dt_ms, int max_points,
                         float* vel_curve_buf, float* out_b, float* out_j, float* out_vb, float* out_vj, int* out_count,
                         float* out_time) {
    float db = b1 - b0;
    float dj = j1 - j0;
    float dist = sqrtf(db * db + dj * dj);

    LinearPathData lpd = {.start_x = b0,
                          .start_y = j0,
                          .end_x = b1,
                          .end_y = j1,
                          .dir_x = (dist > 1e-6f) ? db / dist : 0.0f,
                          .dir_y = (dist > 1e-6f) ? dj / dist : 0.0f};

    Arm2R_PathContext path = {.length = dist, .data = &lpd, .evaluate = _LinearPath};

    return Arm2R_BuildJointTrajectory(&path, vmax, amax, dt_ms, max_points, vel_curve_buf, out_b, out_j, out_vb, out_vj,
                                      out_count, out_time);
}

/**
 * @brief  构建笛卡尔直线路径的关节轨迹
 * @param  b0:            当前 Base 关节位置 (单位: unit)
 * @param  j0:            当前 Joint 关节位置 (单位: unit)
 * @param  x1:            终点 X 坐标
 * @param  y1:            终点 Y 坐标
 * @param  vmax:          路径最大速度
 * @param  amax:          路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  L1:            大臂长度
 * @param  L2:            小臂长度
 * @param  rad_to_base:   弧度转 Base 关节 unit 系数
 * @param  rad_to_joint:  弧度转 Joint 关节 unit 系数
 * @param  joint_off:     Joint 关节安装偏置
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildLine(float b0, float j0, float x1, float y1, float vmax, float amax, int dt_ms,
                        int max_points, float* vel_curve_buf, float L1, float L2, float rad_to_base, float rad_to_joint,
                        float joint_off, float* out_b, float* out_j, float* out_vb, float* out_vj, int* out_count,
                        float* out_time) {
    float x0, y0;
    Arm2R_JointToXY(b0, j0, L1, L2, rad_to_base, rad_to_joint, joint_off, &x0, &y0);

    float dx = x1 - x0;
    float dy = y1 - y0;
    float dist = sqrtf(dx * dx + dy * dy);

    // if (x0 * x1 < 0.0f)
    //     return false;// 机械臂构型改变，现在不考虑象限改变的问题

    LinearPathData lpd = {.start_x = x0,
                          .start_y = y0,
                          .end_x = x1,
                          .end_y = y1,
                          .dir_x = (dist > 1e-6f) ? dx / dist : 0.0f,
                          .dir_y = (dist > 1e-6f) ? dy / dist : 0.0f};

    Arm2R_PathContext path = {.length = dist, .data = &lpd, .evaluate = _LinearPath};

    return Arm2R_BuildCartesianTrajectory(&path, b0, j0, vmax, amax, dt_ms, max_points, vel_curve_buf, L1, L2,
                                          rad_to_base, rad_to_joint, joint_off, out_b, out_j, out_vb, out_vj, out_count,
                                          out_time);
}

/**
 * @brief  构建二阶贝塞尔路径的关节轨迹
 * @param  b0:            当前 Base 关节位置 (单位: unit)
 * @param  j0:            当前 Joint 关节位置 (单位: unit)
 * @param  ctrl_x:        控制点 X 坐标
 * @param  ctrl_y:        控制点 Y 坐标
 * @param  x1:            终点 X 坐标
 * @param  y1:            终点 Y 坐标
 * @param  vmax:          路径最大速度
 * @param  amax:          路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  L1:            大臂长度
 * @param  L2:            小臂长度
 * @param  rad_to_base:   弧度转 Base 关节 unit 系数
 * @param  rad_to_joint:  弧度转 Joint 关节 unit 系数
 * @param  joint_off:     Joint 关节安装偏置
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildBezier2(float b0, float j0, float ctrl_x, float ctrl_y, float x1, float y1,
                           float vmax, float amax, int dt_ms, int max_points, float* vel_curve_buf, float L1, float L2,
                           float rad_to_base, float rad_to_joint, float joint_off, float* out_b, float* out_j,
                           float* out_vb, float* out_vj, int* out_count, float* out_time) {
    float x0, y0;
    Arm2R_JointToXY(b0, j0, L1, L2, rad_to_base, rad_to_joint, joint_off, &x0, &y0);

    Bezier2PathData bpd = {.p0_x = x0, .p0_y = y0, .p1_x = ctrl_x, .p1_y = ctrl_y, .p2_x = x1, .p2_y = y1};

    // 预估曲线总长度
    float dist = _Bezier2Length(&bpd, 100);

    Arm2R_PathContext path = {.length = dist, .data = &bpd, .evaluate = _Bezier2Path};

    return Arm2R_BuildCartesianTrajectory(&path, b0, j0, vmax, amax, dt_ms, max_points, vel_curve_buf, L1, L2,
                                          rad_to_base, rad_to_joint, joint_off, out_b, out_j, out_vb, out_vj, out_count,
                                          out_time);
}

/**
 * @brief  构建三阶贝塞尔路径的关节轨迹
 * @param  b0:            当前 Base 关节位置 (单位: unit)
 * @param  j0:            当前 Joint 关节位置 (单位: unit)
 * @param  ctrl1_x:       第一控制点 X 坐标
 * @param  ctrl1_y:       第一控制点 Y 坐标
 * @param  ctrl2_x:       第二控制点 X 坐标
 * @param  ctrl2_y:       第二控制点 Y 坐标
 * @param  x1:            终点 X 坐标
 * @param  y1:            终点 Y 坐标
 * @param  vmax:          路径最大速度
 * @param  amax:          路径最大加速度
 * @param  dt_ms:         轨迹采样周期 (单位: ms)
 * @param  max_points:    输出数组容量
 * @param  vel_curve_buf: 速度曲线缓存数组
 * @param  L1:            大臂长度
 * @param  L2:            小臂长度
 * @param  rad_to_base:   弧度转 Base 关节 unit 系数
 * @param  rad_to_joint:  弧度转 Joint 关节 unit 系数
 * @param  joint_off:     Joint 关节安装偏置
 * @param  out_b:         输出 Base 位置数组
 * @param  out_j:         输出 Joint 位置数组
 * @param  out_vb:        输出 Base 速度数组
 * @param  out_vj:        输出 Joint 速度数组
 * @param  out_count:     输出轨迹点数指针
 * @param  out_time:      输出轨迹总时间指针，允许为空
 * @retval 0: 成功, 非0: 轨迹构建失败
 */
uint8_t Arm2R_BuildBezier3(float b0, float j0, float ctrl1_x, float ctrl1_y, float ctrl2_x, float ctrl2_y, float x1,
                           float y1, float vmax, float amax, int dt_ms, int max_points,
                           float* vel_curve_buf, float L1, float L2, float rad_to_base, float rad_to_joint,
                           float joint_off, float* out_b, float* out_j, float* out_vb, float* out_vj, int* out_count,
                           float* out_time) {
    float x0, y0;
    Arm2R_JointToXY(b0, j0, L1, L2, rad_to_base, rad_to_joint, joint_off, &x0, &y0);

    Bezier3PathData bpd = {.p0_x = x0,
                           .p0_y = y0,
                           .p1_x = ctrl1_x,
                           .p1_y = ctrl1_y,
                           .p2_x = ctrl2_x,
                           .p2_y = ctrl2_y,
                           .p3_x = x1,
                           .p3_y = y1};

    // 预估曲线总长度
    float dist = _Bezier3Length(&bpd, 100);

    Arm2R_PathContext path = {.length = dist, .data = &bpd, .evaluate = _Bezier3Path};

    return Arm2R_BuildCartesianTrajectory(&path, b0, j0, vmax, amax, dt_ms, max_points, vel_curve_buf, L1, L2,
                                          rad_to_base, rad_to_joint, joint_off, out_b, out_j, out_vb, out_vj, out_count,
                                          out_time);
}

