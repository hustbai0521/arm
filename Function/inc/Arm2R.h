/**
 * @file    Arm2R.h
 * @brief   Planar 2R arm kinematics and trajectory planning header file
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-18
 */
#ifndef ARM2R_H
#define ARM2R_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Arm2R 数值计算宏定义
 */
#define ARM_EPSILON 1e-6f                   ///< 浮点零值判断阈值
#define ARM2R_IK_TOLERANCE 2.0f             ///< 逆解关节边界容差 (单位: unit)
#define ARM2R_SPEED_PLAN_MIN_FACTOR 10.0f   ///< 速度规划中的最低速度绝对值
#define ARM2R_SPEED_PLAN_ACCEL_FACTOR 6.0f  ///< 显式加速段启用时的加速段系数
/**
 * @brief 速度规划是否显式考虑加速段
 * @note  0: 保持原行为，仅显式处理减速段
 * @note  1: 在速度规划中加入加速-巡航-减速三段式
 */
#define ARM2R_SPEED_PLAN_INCLUDE_ACCEL 0

/**
 * @brief Arm2R 路径上下文结构体
 */
typedef struct _Arm2R_PathContext {
    float length;  ///< 路径近似总长度
    void* data;    ///< 路径私有数据指针

    /**
     * @brief  路径采样回调函数
     * @param  ctx:       路径上下文指针
     * @param  u:         路径归一化进度 (范围: 0.0~1.0)
     * @param  out_x:     输出 X 坐标指针
     * @param  out_y:     输出 Y 坐标指针
     * @param  out_dir_x: 输出切向量 X 分量指针
     * @param  out_dir_y: 输出切向量 Y 分量指针
     * @retval true: 采样成功, false: 采样失败
     */
    bool (*evaluate)(struct _Arm2R_PathContext* ctx, float u, float* out_x, float* out_y, float* out_dir_x,
                     float* out_dir_y);
} Arm2R_PathContext;

/* ==================== 笛卡尔轨迹构建接口 ==================== */

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
                        float* out_time);

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
                           float* out_time);

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
                           float* out_vb, float* out_vj, int* out_count, float* out_time);

/* ==================== 关节轨迹构建接口 ==================== */

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
                         float* out_time);

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
                                       float* out_vj, int* out_count, float* out_time);

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
                                   int* out_count, float* out_time);

/* ==================== 运动学与速度规划接口 ==================== */

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
bool Arm2R_SpeedPlan(float dist, float vmax, float amax, float* vel_curve, int max_points, float* out_total_time);

/**
 * @brief  根据速度曲线生成单个插值点
 * @param  vel_curve:    速度曲线数组
 * @param  max_points:   速度曲线数组容量
 * @param  total_time:   规划总时间
 * @param  dt_ms:        步进时间 (单位: ms)
 * @param  current_time: 输入输出当前累计时间指针
 * @param  current_s:    输入输出当前累计位移指针
 * @param  out_s:        输出本步位移指针
 * @param  out_v:        输出本步瞬时速度指针
 * @retval true: 还有下一个点, false: 已到达终点
 */
bool Arm2R_CreatePoint(const float* vel_curve, int max_points, float total_time, float dt_ms, float* current_time,
                       float* current_s, float* out_s, float* out_v);

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
void Arm2R_FK(float b, float j, float L1, float L2, float base_to_deg, float joint_to_deg, float joint_off,
              float deg_to_rad, float* x, float* y);

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
bool Arm2R_IK(float x, float y, float L1, float L2, float rad_to_base, float rad_to_joint, float joint_off,
              bool clamp_radius, float* b1, float* j1, float* b2, float* j2, bool* v1, bool* v2);

#endif /* ARM2R_H */

