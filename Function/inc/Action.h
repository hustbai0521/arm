/**
 * @file    Action.h
 * @brief   Robot action configuration interface
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-30
 *
 * @note
 * 本文件只描述当前 arm 工程的业务动作，不实现队列调度算法。
 * 通用队列、步骤、资源锁和运行时接口均由 Action_Lib 提供。
 */

#ifndef ACTION_H
#define ACTION_H

#include "Action_Lib.h"

/* ========================================================================== 
 * 实例内端口
 *
 * target 用于同一个设备实例内部的子通道路由。大臂由脉塔电机驱动，
 * 小臂由达妙电机驱动。
 * ========================================================================== */

typedef enum {
    ACTION_PORT_BIG_ARM = 0,    // 脉塔，大臂
    ACTION_PORT_SMALL_ARM,      // 达妙，小臂
    ACTION_PORT_ARM_TRAJECTORY, // 双关节 TP_JOINT 轨迹
} ActionPort;

/* ========================================================================== 
 * 设备操作名
 *
 * op 是 Action_Lib 与设备适配层之间的轻量协议：
 * position 的双浮点参数依次为目标位置(rad)和速度(rad/s)；
 * 脉塔电机使用最大速度，达妙 MIT 位置控制使用目标速度。
 * ========================================================================== */

#define ACTION_OP_POSITION "position"
#define ACTION_OP_TP_JOINT "tp_joint"
#define ACTION_OP_STOP "stop"

/* ========================================================================== 
 * 动作组 ID
 *
 * ACTION_GROUP_NULL 同时是动作队列的结束标志，必须保持为 0。
 * ARM_LIFT 同时控制大臂脉塔电机和小臂达妙电机。
 * ========================================================================== */

enum {
    ACTION_GROUP_NULL = 0,
    ACTION_GROUP_ARM_STOP,
    ACTION_GROUP_ARM_LIFT,
    ACTION_GROUP_ARM_FOLD,
};

typedef ActionId ActionGroup;

/* ========================================================================== 
 * 动作运行实例 ID
 *
 * 一个 ID 对应一条独立动作队列。
 * ========================================================================== */

typedef enum {
    ACTION_TARGET_ARM = 0,
    ACTION_TARGET_COUNT,
} ActionTarget;

/* ========================================================================== 
 * 通用工具宏
 * ========================================================================== */

#define MAX_ACTION_GROUP_QUEUE_SIZE ACTION_GROUP_QUEUE_MAX

/* ========================================================================== 
 * ARM 命令 / 步骤构造宏
 *
 * 这些宏只负责声明静态动作数据，不直接访问电机。实际命令由 Action.c 中的
 * Arm_Exec 执行，完成条件由 Arm_Is_Done 判断。
 * ========================================================================== */

/**
 * @brief 构造大臂脉塔电机位置命令
 * @param position_v 目标绝对位置，单位 rad
 * @param speed_v    最大速度，单位 rad/s，必须为正数
 */
#define CMD_BIG_ARM_POSITION(position_v, speed_v)                      \
    ACT_CMD_PAIR(ACTION_PORT_BIG_ARM, ACTION_OP_POSITION,              \
                 (position_v), (speed_v))

/**
 * @brief 构造大臂脉塔电机位置到达等待条件
 * @note  speed_v 保留在等待参数中，使步骤命令与等待描述保持一致；完成判断只比较位置。
 */
#define WAIT_BIG_ARM_POSITION(position_v, speed_v)                     \
    ACT_WAIT_PAIR(ACTION_PORT_BIG_ARM, ACTION_OP_POSITION,             \
                  (position_v), (speed_v))

/** @brief 构造小臂达妙电机 MIT 位置命令。 */
#define CMD_SMALL_ARM_POSITION(position_v, speed_v)                    \
    ACT_CMD_PAIR(ACTION_PORT_SMALL_ARM, ACTION_OP_POSITION,            \
                 (position_v), (speed_v))

/** @brief 构造小臂达妙电机位置到达等待条件。 */
#define WAIT_SMALL_ARM_POSITION(position_v, speed_v)                   \
    ACT_WAIT_PAIR(ACTION_PORT_SMALL_ARM, ACTION_OP_POSITION,           \
                  (position_v), (speed_v))

/**
 * @brief 构造以关节角为输入的 TP_JOINT 轨迹命令
 * @param theta1_deg_v 大臂相对 X 轴的绝对角度，单位 deg
 * @param theta2_deg_v 小臂相对大臂的关节角，单位 deg
 */
#define CMD_ARM_TP_JOINT(theta1_deg_v, theta2_deg_v)                    \
    ACT_CMD_PAIR(ACTION_PORT_ARM_TRAJECTORY, ACTION_OP_TP_JOINT,       \
                 (theta1_deg_v), (theta2_deg_v))

/** @brief 等待 TP_JOINT 轨迹结束且两台电机实际到位。 */
#define WAIT_ARM_TP_JOINT(theta1_deg_v, theta2_deg_v)                   \
    ACT_WAIT_PAIR(ACTION_PORT_ARM_TRAJECTORY, ACTION_OP_TP_JOINT,      \
                  (theta1_deg_v), (theta2_deg_v))

/**
 * @brief 构造大臂脉塔电机停止命令
 */
#define CMD_BIG_ARM_STOP()                                             \
    {                                                                  \
        .target = ACTION_PORT_BIG_ARM, .op = ACTION_OP_STOP,           \
        .value = ACT_VAL_NONE()                                        \
    }

/** @brief 构造小臂达妙电机停止命令。 */
#define CMD_SMALL_ARM_STOP()                                           \
    {                                                                  \
        .target = ACTION_PORT_SMALL_ARM, .op = ACTION_OP_STOP,         \
        .value = ACT_VAL_NONE()                                        \
    }

/**
 * @brief 构造一个位置动作步骤
 * @param position_v 目标绝对位置，单位 rad
 * @param speed_v    最大速度，单位 rad/s
 * @param dly_v      到位后的延时 tick
 */
#define STP_BIG_ARM_POSITION(position_v, speed_v, dly_v)               \
    {                                                                  \
        .commands = {CMD_BIG_ARM_POSITION(position_v, speed_v)},       \
        .waits = {WAIT_BIG_ARM_POSITION(position_v, speed_v)},         \
        .dly = (dly_v),                                                \
    }

/**
 * @brief 构造一个停止步骤
 * @param dly_v 停止命令后的延时 tick
 */
#define STP_ARM_STOP(dly_v)                                            \
    {                                                                  \
        .commands = {CMD_BIG_ARM_STOP(), CMD_SMALL_ARM_STOP()},        \
        .command_count = 2U,                                           \
        .dly = (dly_v),                                                \
    }

/**
 * @brief 构造一个完整的 TP_JOINT 轨迹步骤
 * @note 可在同一个 ActionStep 数组中多次使用，以顺序运行多个关节目标点。
 */
#define STP_ARM_TP_JOINT(theta1_deg_v, theta2_deg_v, dly_v)             \
    {                                                                  \
        .commands = {CMD_ARM_TP_JOINT((theta1_deg_v),                  \
                                      (theta2_deg_v))},                \
        .waits = {WAIT_ARM_TP_JOINT((theta1_deg_v),                    \
                                    (theta2_deg_v))},                  \
        .dly = (dly_v),                                                \
    }

#endif /* ACTION_H */
