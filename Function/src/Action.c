/**
 * @file    Action.c
 * @brief   Project action catalog, device class and device instance registration
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-30
 *
 * @note
 * 本文件沿用 ArmPlatform_H7 的四层组织方式：
 * 动作步骤 -> 动作组目录 -> 设备类 -> 设备实例/运行时注册。
 * 与旧工程不同的是，这里只保留当前 arm 工程的新 Arm 设备，不再引用
 * ArmPlatform、Rod、Combine、Cylinder 等旧机构类型。
 */

#include "Action.h"
#include "ArmTrajectory.h"
#include "Task_config.h"
#include "cmsis_os2.h"
#include "damiao_comm_port.h"
#include "myactuator_comm_port.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

/** 位置动作的默认完成误差，单位 rad；应在新机构完成实测后重新标定。 */
#define ARM_POSITION_OK_RANGE 0.02f

/** 大臂轨迹点使用脉塔 Motion 模式；先以保守刚度/阻尼作为实机调参起点。 */
#define ARM_LIFT_BIG_CONTROL_MODE MYACTUATOR_CTRL_MOTION
#define ARM_LIFT_BIG_TORQUE 0.05f
#define ARM_LIFT_BIG_KP 45.0f
#define ARM_LIFT_BIG_KD 0.5f

/** 小臂轨迹点使用达妙 MIT 位置 + 速度前馈控制。 */
#define ARM_DAMIAO_POSITION_TORQUE 0.0f
#define ARM_DAMIAO_POSITION_KP 40.0f
#define ARM_DAMIAO_POSITION_KD 0.5f
#define ARM_STARTUP_COMMAND_PERIOD_TICKS 20U
#define ARM_STARTUP_BIG_VELOCITY_RAD_S 0.0f

/* ========================================================================== 
 * 本地类型
 *
 * ActionArmDevice 是动作层到现有任务/驱动层的最小适配器：
 * command 是 MotorTask 消费的命令邮箱，motor 提供连接状态和位置反馈。
 * 该类型只在本文件可见，避免把具体电机协议泄漏到 Action.h。
 * ========================================================================== */

typedef struct {
    volatile MYACTUATOR_TestCommand* big_command;
    MYACTUATOR_Motor* big_motor;
    volatile DAMIAO_TestCommand* small_command;
    DamiaoMotor* small_motor;
    float big_position_tolerance;
    float small_position_tolerance;
    ArmTrajectory trajectory;
    bool trajectory_initialized;
    bool trajectory_start_pending;
    float pending_theta1_deg;
    float pending_theta2_deg;
    float startup_big_hold_position;
    float startup_small_hold_position;
    uint32_t startup_command_tick;
} ActionArmDevice;

/* ========================================================================== 
 * 动作步骤定义
 *
 * 每个步骤直接给出 theta1/theta2，轨迹层内部换算目标 X/Y。等待阶段每 1 ms
 * 用实时反馈推进最近点
 * 跟踪，并同步更新两台电机的轨迹位置；轨迹到达终点且两轴实际到位才完成。
 * ========================================================================== */

static const ActionStep ARM_STOP[] = {
    STP_ARM_STOP(0U),
};

static const ActionStep ARM_LIFT[] = {
	  STP_ARM_TP_JOINT(45.0f, 90.0f, 1000U),
	  STP_ARM_TP_JOINT(70.0f, -70.0f, 1000U),
	  STP_ARM_TP_JOINT(0.0f, 90.0f, 1000U),
	  STP_ARM_TP_JOINT(70.0f, -120.0f, 0U),
	
};

/* ========================================================================== 
 * 动作组资源标签
 *
 * local tag 用于同一 Arm 实例上的队列互斥。以后即使为同一设备增加多条
 * 执行队列，带有 "arm" 标签的动作也不会并发争用电机。
 * ========================================================================== */

static const char* const LOCAL_TAGS_ARM[] = {"arm"};

#define ARM_GROUP(id_v, step_arr)                                     \
    ACTION_GROUP_ENTRY(id_v, step_arr, ACTION_NO_TAGS,                 \
                       ACTION_TAGS(LOCAL_TAGS_ARM))

/* ========================================================================== 
 * 分设备动作组
 *
 * 这里只登记当前设备真正支持的业务动作。后续增加动作时，先定义 ActionStep
 * 数组，再把动作组 ID 与数组登记到本目录中。
 * ========================================================================== */

static const ActionEntry Arm_Action_Groups[] = {
    ARM_GROUP(ACTION_GROUP_ARM_STOP, ARM_STOP),
    ARM_GROUP(ACTION_GROUP_ARM_LIFT, ARM_LIFT),
};

/* ========================================================================== 
 * 新 Arm 设备适配逻辑
 *
 * Action_Lib 只认识 void* ctx 和通用 ActionCommand；以下函数负责把它们
 * 转换为 MYACTUATOR/MotorTask 能理解的状态。这样队列引擎不依赖具体电机协议。
 * ========================================================================== */

/**
 * @brief 检查 Arm 实例是否已经完成基本初始化
 * @note connected 不作为启动条件；刚上电尚未收到首帧反馈时仍允许下发停止命令。
 */
static bool Arm_Is_Available(void* ctx) {
    ActionArmDevice* arm = (ActionArmDevice*)ctx;

    if (arm != NULL && !arm->trajectory_initialized) {
        ArmTrajectory_Init(&arm->trajectory);
        arm->trajectory_initialized = true;
    }

    return arm != NULL && arm->big_command != NULL &&
           arm->big_motor != NULL && arm->small_command != NULL &&
           arm->small_motor != NULL &&
           arm->big_motor->id >= MYACTUATOR_CAN_ID_MIN &&
           arm->big_motor->id <= MYACTUATOR_CAN_ID_MAX &&
           arm->small_motor->id <= 0x0FU &&
           arm->small_motor->mst_id <= 0x7FFU;
}

/** @brief 开始更新命令邮箱，并返回本次写入使用的奇数版本。 */
static uint32_t Arm_BeginCommandUpdate(volatile uint32_t* sequence) {
    uint32_t updating_sequence = (*sequence + 1U) | 1U;

    *sequence = updating_sequence;
    /* 保证“写入中”状态先于后续命令字段对另一个任务可见。 */
    __DMB();
    return updating_sequence;
}

/** @brief 所有命令字段写完后，以偶数版本发布完整命令。 */
static void Arm_EndCommandUpdate(volatile uint32_t* sequence,
                                 uint32_t updating_sequence) {
    /* 保证命令字段先写完，再让 MotorTask 看见新的稳定版本。 */
    __DMB();
    *sequence = updating_sequence + 1U;
}

/**
 * @brief 发布一版完整的大臂脉塔命令
 * @note POSITION 与 MOTION 共用此入口；各模式不用的字段仍显式清零或覆盖，
 *       避免切换模式时沿用上一条命令的参数。
 * @note 当前只允许 ActionTask 作为命令邮箱发布者；如增加多个发布者，应改用消息队列。
 */
static void Arm_PublishBigCommand(ActionArmDevice* arm,
                                  MYACTUATOR_ControlMode control_mode,
                                  float position,
                                  float speed,
                                  float torque,
                                  float kp,
                                  float kd) {
    volatile MYACTUATOR_TestCommand* command = arm->big_command;
    uint32_t updating_sequence = Arm_BeginCommandUpdate(&command->sequence);

    command->enabled = 0U;
    command->control_mode = control_mode;
    command->position = position;
    command->speed = speed;
    command->torque = torque;
    command->kp = kp;
    command->kd = kd;
    command->enabled = 1U;
    Arm_EndCommandUpdate(&command->sequence, updating_sequence);
}

/** @brief 校验大臂脉塔 POSITION/MOTION 命令的动作层参数。 */
static bool Arm_IsBigCommandValid(MYACTUATOR_ControlMode control_mode,
                                  float position,
                                  float speed,
                                  float torque,
                                  float kp,
                                  float kd) {
    if (!isfinite(position) || !isfinite(speed) || !isfinite(torque) ||
        !isfinite(kp) || !isfinite(kd)) {
        return false;
    }

    if (control_mode == MYACTUATOR_CTRL_POSITION) {
        return fabsf(position) <= 374000.0f &&
               speed >= 0.017454f && speed <= 1143.8f;
    }

    if (control_mode == MYACTUATOR_CTRL_MOTION) {
        return position >= MYACTUATOR_MOTION_P_MIN &&
               position <= MYACTUATOR_MOTION_P_MAX &&
               speed >= MYACTUATOR_MOTION_V_MIN &&
               speed <= MYACTUATOR_MOTION_V_MAX &&
               torque >= MYACTUATOR_MOTION_T_MIN &&
               torque <= MYACTUATOR_MOTION_T_MAX &&
               kp >= MYACTUATOR_MOTION_KP_MIN &&
               kp <= MYACTUATOR_MOTION_KP_MAX &&
               kd >= MYACTUATOR_MOTION_KD_MIN &&
               kd <= MYACTUATOR_MOTION_KD_MAX;
    }

    return false;
}

/** @brief 发布一版完整的小臂达妙 MIT 位置命令。 */
static void Arm_PublishSmallPositionCommand(ActionArmDevice* arm,
                                             float position,
                                             float speed) {
    volatile DAMIAO_TestCommand* command = arm->small_command;
    uint32_t updating_sequence = Arm_BeginCommandUpdate(&command->sequence);

    command->enabled = 0U;
    command->motor_type = arm->small_motor->type;
    command->control_mode = DAMIAO_CTRL_MIT;
    command->position = position;
    command->speed = speed;
    command->torque = ARM_DAMIAO_POSITION_TORQUE;
    command->kp = ARM_DAMIAO_POSITION_KP;
    command->kd = ARM_DAMIAO_POSITION_KD;
    command->enabled = 1U;
    Arm_EndCommandUpdate(&command->sequence, updating_sequence);
}

/** @brief 用 Motion 保持帧使能脉塔，并通过其应答取得新鲜位置反馈。 */
static void Arm_PublishBigStartupCommand(ActionArmDevice* arm) {
    Arm_PublishBigCommand(arm,
                          ARM_LIFT_BIG_CONTROL_MODE,
                          arm->startup_big_hold_position,
                          ARM_STARTUP_BIG_VELOCITY_RAD_S,
                          ARM_LIFT_BIG_TORQUE,
                          ARM_LIFT_BIG_KP,
                          ARM_LIFT_BIG_KD);
}

/** @brief 用 MIT 位置模式首帧使能达妙，并通过其应答取得新鲜位置反馈。 */
static void Arm_PublishSmallStartupCommand(ActionArmDevice* arm) {
    volatile DAMIAO_TestCommand* command = arm->small_command;
    uint32_t updating_sequence = Arm_BeginCommandUpdate(&command->sequence);

    command->enabled = 0U;
    command->motor_type = arm->small_motor->type;
    command->control_mode = DAMIAO_CTRL_MIT;
    command->esc_id = arm->small_motor->id;
    command->master_id = arm->small_motor->mst_id;
    command->position = arm->startup_small_hold_position;
    command->speed = 0.0f;
    command->torque = ARM_DAMIAO_POSITION_TORQUE;
    command->kp = ARM_DAMIAO_POSITION_KP;
    command->kd = ARM_DAMIAO_POSITION_KD;
    command->current = 0.0f;
    command->enabled = 1U;
    Arm_EndCommandUpdate(&command->sequence, updating_sequence);
}

static bool Arm_HasTrajectoryFeedback(const ActionArmDevice* arm) {
    return arm->big_motor->flag.connected != 0U &&
           arm->big_motor->flag.feedback_ready != 0U &&
           arm->small_motor->flag.connected != 0U &&
           arm->small_motor->flag.feedback_ready != 0U;
}

/** @brief 周期发送两台电机各自的运行模式帧，直到两轴反馈都已就绪。 */
static void Arm_PublishStartupCommands(ActionArmDevice* arm,
                                       uint32_t now_tick) {
    if ((uint32_t)(now_tick - arm->startup_command_tick) <
        ARM_STARTUP_COMMAND_PERIOD_TICKS) {
        return;
    }

    arm->startup_command_tick = now_tick;
    Arm_PublishBigStartupCommand(arm);
    Arm_PublishSmallStartupCommand(arm);
}

/** @brief 同步发布一个 TP_JOINT 轨迹点到两台电机。 */
static bool Arm_PublishTrajectoryCommand(
    ActionArmDevice* arm,
    const ArmTrajectoryMotorCommand* trajectory_command) {
    const DamiaoMotorLimit* limit;

    if (arm == NULL || trajectory_command == NULL)
        return false;

    limit = &arm->small_motor->limit;
    if (!Arm_IsBigCommandValid(ARM_LIFT_BIG_CONTROL_MODE,
                               trajectory_command->big_position_rad,
                               trajectory_command->big_speed_rad_s,
                               ARM_LIFT_BIG_TORQUE,
                               ARM_LIFT_BIG_KP,
                               ARM_LIFT_BIG_KD) ||
        DamiaoMotor_IsTypeSupported(arm->small_motor->type) == 0U ||
        trajectory_command->small_position_rad < limit->p_min ||
        trajectory_command->small_position_rad > limit->p_max ||
        trajectory_command->small_speed_rad_s < limit->v_min ||
        trajectory_command->small_speed_rad_s > limit->v_max) {
        return false;
    }

    Arm_PublishBigCommand(arm,
                          ARM_LIFT_BIG_CONTROL_MODE,
                          trajectory_command->big_position_rad,
                          trajectory_command->big_speed_rad_s,
                          ARM_LIFT_BIG_TORQUE,
                          ARM_LIFT_BIG_KP,
                          ARM_LIFT_BIG_KD);
    Arm_PublishSmallPositionCommand(arm,
                                    trajectory_command->small_position_rad,
                                    trajectory_command->small_speed_rad_s);
    return true;
}

/** @brief 发布大臂脉塔安全停止命令。 */
static void Arm_PublishBigStopCommand(ActionArmDevice* arm) {
    volatile MYACTUATOR_TestCommand* command = arm->big_command;
    uint32_t updating_sequence = Arm_BeginCommandUpdate(&command->sequence);

    command->enabled = 0U;
    command->control_mode = MYACTUATOR_CTRL_STOP;
    Arm_EndCommandUpdate(&command->sequence, updating_sequence);
}

/** @brief 发布小臂达妙安全停止命令。 */
static void Arm_PublishSmallStopCommand(ActionArmDevice* arm) {
    volatile DAMIAO_TestCommand* command = arm->small_command;
    uint32_t updating_sequence = Arm_BeginCommandUpdate(&command->sequence);

    command->enabled = 0U;
    command->control_mode = DAMIAO_CTRL_STOP;
    Arm_EndCommandUpdate(&command->sequence, updating_sequence);
}

/**
 * @brief 执行动作步骤中的单条设备命令
 */
static bool Arm_Exec(void* ctx,
                     ActionQueue* queue,
                     const ActionCommand* cmd) {
    ActionArmDevice* arm = (ActionArmDevice*)ctx;

    (void)queue;
    if (arm == NULL || cmd == NULL || cmd->op == NULL)
        return false;

    if (Action_StrEq(cmd->op, ACTION_OP_STOP)) {
        if (cmd->value.type != ACTION_VALUE_NONE)
            return false;

        if (cmd->target == ACTION_PORT_BIG_ARM) {
            ArmTrajectory_Abort(&arm->trajectory);
            Arm_PublishBigStopCommand(arm);
            return true;
        }
        if (cmd->target == ACTION_PORT_SMALL_ARM) {
            ArmTrajectory_Abort(&arm->trajectory);
            Arm_PublishSmallStopCommand(arm);
            return true;
        }
        return false;
    }

    if (Action_StrEq(cmd->op, ACTION_OP_TP_JOINT)) {
        uint32_t now_tick;

        if (cmd->target != ACTION_PORT_ARM_TRAJECTORY ||
            cmd->value.type != ACTION_VALUE_PAIR_FLOAT ||
            !isfinite(cmd->value.data.pair.a) ||
            !isfinite(cmd->value.data.pair.b)) {
            return false;
        }

        ArmTrajectory_Abort(&arm->trajectory);
        arm->trajectory_start_pending = true;
        arm->pending_theta1_deg = cmd->value.data.pair.a;
        arm->pending_theta2_deg = cmd->value.data.pair.b;
        arm->startup_big_hold_position =
            (arm->big_motor->flag.feedback_ready != 0U)
                ? arm->big_motor->rx_Pos
                : ARM_TRAJECTORY_BIG_ZERO_RAD;
        arm->startup_small_hold_position =
            (arm->small_motor->flag.feedback_ready != 0U)
                ? arm->small_motor->rx_Pos
                : ARM_TRAJECTORY_SMALL_ZERO_RAD;
        now_tick = osKernelGetTickCount();
        arm->startup_command_tick = now_tick - ARM_STARTUP_COMMAND_PERIOD_TICKS;

        /*
         * 收到许可后立即把两台电机切入各自运行模式；两轴反馈均就绪后，
         * 使用当前实测位置作为 TP_JOINT 的轨迹起点。
         */
        Arm_PublishStartupCommands(arm, now_tick);
        return true;
    }

    if (Action_StrEq(cmd->op, ACTION_OP_POSITION)) {
        float position;
        float speed;

        if (cmd->value.type != ACTION_VALUE_PAIR_FLOAT)
            return false;

        position = cmd->value.data.pair.a;
        speed = cmd->value.data.pair.b;
        if (!isfinite(position) || !isfinite(speed))
            return false;

        if (cmd->target == ACTION_PORT_BIG_ARM) {
            if (!Arm_IsBigCommandValid(ARM_LIFT_BIG_CONTROL_MODE,
                                       position,
                                       speed,
                                       ARM_LIFT_BIG_TORQUE,
                                       ARM_LIFT_BIG_KP,
                                       ARM_LIFT_BIG_KD)) {
                return false;
            }
            Arm_PublishBigCommand(arm,
                                  ARM_LIFT_BIG_CONTROL_MODE,
                                  position,
                                  speed,
                                  ARM_LIFT_BIG_TORQUE,
                                  ARM_LIFT_BIG_KP,
                                  ARM_LIFT_BIG_KD);
            return true;
        }

        if (cmd->target == ACTION_PORT_SMALL_ARM) {
            const DamiaoMotorLimit* limit = &arm->small_motor->limit;

            if (DamiaoMotor_IsTypeSupported(arm->small_motor->type) == 0U ||
                position < limit->p_min || position > limit->p_max ||
                speed < limit->v_min || speed > limit->v_max)
                return false;
            Arm_PublishSmallPositionCommand(arm, position, speed);
            return true;
        }
    }

    return false;
}

/**
 * @brief 检查动作步骤中的等待条件
 */
static bool Arm_Is_Done(void* ctx,
                        ActionQueue* queue,
                        const ActionWaitCondition* wait) {
    ActionArmDevice* arm = (ActionArmDevice*)ctx;
    float expected_position;

    (void)queue;
    if (arm == NULL || wait == NULL || wait->op == NULL)
        return false;

    if (Action_StrEq(wait->op, ACTION_OP_STOP)) {
        if (wait->target == ACTION_PORT_BIG_ARM)
            return arm->big_command->enabled == 0U;
        if (wait->target == ACTION_PORT_SMALL_ARM)
            return arm->small_command->enabled == 0U;
        return false;
    }

    if (Action_StrEq(wait->op, ACTION_OP_TP_JOINT)) {
        ArmTrajectoryMotorCommand trajectory_command;
        float final_big_position;
        float final_small_position;
        uint32_t now_tick = osKernelGetTickCount();

        if (wait->target != ACTION_PORT_ARM_TRAJECTORY ||
            wait->expected.type != ACTION_VALUE_PAIR_FLOAT) {
            return false;
        }

        if (arm->trajectory_start_pending) {
            if (!Arm_HasTrajectoryFeedback(arm)) {
                Arm_PublishStartupCommands(arm, now_tick);
                return false;
            }

            if (!ArmTrajectory_StartJointAngles(&arm->trajectory,
                                                 arm->pending_theta1_deg,
                                                 arm->pending_theta2_deg,
                                                 arm->big_motor->rx_Pos,
                                                 arm->small_motor->rx_Pos,
                                                 now_tick)) {
                ArmTrajectory_Abort(&arm->trajectory);
                arm->trajectory_start_pending = false;
                return false;
            }
            arm->trajectory_start_pending = false;
        }

        if (!Arm_HasTrajectoryFeedback(arm) ||
            !ArmTrajectory_Update(&arm->trajectory,
                                  arm->big_motor->rx_Pos,
                                  arm->small_motor->rx_Pos,
                                  now_tick,
                                  &trajectory_command) ||
            !Arm_PublishTrajectoryCommand(arm, &trajectory_command)) {
            return false;
        }

        if (arm->trajectory.State != ARM_TP_IDLE)
            return false;

        ArmTrajectory_GetFinalMotorPosition(&arm->trajectory,
                                            &final_big_position,
                                            &final_small_position);
        return fabsf(arm->big_motor->rx_Pos - final_big_position) <=
                   arm->big_position_tolerance &&
               fabsf(arm->small_motor->rx_Pos - final_small_position) <=
                   arm->small_position_tolerance;
    }

    if (!Action_StrEq(wait->op, ACTION_OP_POSITION) ||
        wait->expected.type != ACTION_VALUE_PAIR_FLOAT)
        return false;

    expected_position = wait->expected.data.pair.a;
    if (wait->target == ACTION_PORT_BIG_ARM) {
        if (arm->big_motor->flag.connected == 0U ||
            arm->big_motor->flag.feedback_ready == 0U)
            return false;
        return fabsf(arm->big_motor->rx_Pos - expected_position) <=
               arm->big_position_tolerance;
    }

    if (wait->target == ACTION_PORT_SMALL_ARM) {
        if (arm->small_motor->flag.connected == 0U ||
            arm->small_motor->flag.feedback_ready == 0U)
            return false;
        return fabsf(arm->small_motor->rx_Pos - expected_position) <=
               arm->small_position_tolerance;
    }

    return false;
}

/**
 * @brief 复位动作实例时让 Arm 回到安全停止态
 */
static void Arm_ActReset(void* ctx) {
    ActionArmDevice* arm = (ActionArmDevice*)ctx;

    if (arm == NULL)
        return;

    arm->trajectory_start_pending = false;
    ArmTrajectory_Abort(&arm->trajectory);
    if (arm->big_command != NULL)
        Arm_PublishBigStopCommand(arm);
    if (arm->small_command != NULL)
        Arm_PublishSmallStopCommand(arm);
}

/* ========================================================================== 
 * 设备类注册
 *
 * 设备类绑定动作组目录和适配函数，但不代表某个具体物理实例。
 * ========================================================================== */

static const ActionDevice Arm_Device_Class = {
    .entries = Arm_Action_Groups,
    .entry_count = ACTION_ARRAY_SIZE(Arm_Action_Groups),
    .null_id = ACTION_GROUP_NULL,
    .is_available = Arm_Is_Available,
    .exec = Arm_Exec,
    .is_done = Arm_Is_Done,
    .reset = Arm_ActReset,
};

/* ========================================================================== 
 * 设备实例注册
 *
 * 当前 Arm 实例复用已有的 MotorTask 命令邮箱和 MYACTUATOR 反馈对象。
 * 将来引入独立 Arm 设备模块时，只需替换此处 ctx，队列引擎无需改动。
 * ========================================================================== */

static ActionArmDevice Global_Action_Arm = {
    .big_command = &myactuator_test_command,
    .big_motor = &myactuator_x4_36,
    .small_command = &damiao_test_command,
    .small_motor = &damiao_motor,
    .big_position_tolerance = ARM_POSITION_OK_RANGE,
    .small_position_tolerance = ARM_POSITION_OK_RANGE,
    .trajectory_initialized = false,
    .trajectory_start_pending = false,
};

/* ========================================================================== 
 * 动作运行实例注册
 *
 * Action_Handles 是唯一运行注册表：每项同时绑定 id、queue 和设备实例。
 * ========================================================================== */

#define ACTION_QUEUE_INIT                                               \
    {                                                                   \
        .GroupState = ACTION_STATE_IDLE, .ActState = ACTION_STATE_IDLE, \
        .open_wait = true                                               \
    }

static const ActionDeviceInstance Arm_Device_Instance = {
    .device_class = &Arm_Device_Class,
    .ctx = &Global_Action_Arm,
};

ActionExecHandle Action_Handles[] = {
    [ACTION_TARGET_ARM] = {
        .id = ACTION_TARGET_ARM,
        .queue = ACTION_QUEUE_INIT,
        .instance = &Arm_Device_Instance,
    },
};

ActionRuntime Action_Runtime = {
    .exec_handles = Action_Handles,
    .exec_handle_count = ACTION_ARRAY_SIZE(Action_Handles),
};
