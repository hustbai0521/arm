#include "Task_config.h"

#include "Action.h"
#include "cmsis_os2.h"
#include "damiao_comm_port.h"
#include "motor_comm_port.h"
#include "myactuator_comm_port.h"
#include "robstride_comm_port.h"

/*
 * LIFT 的外部触发许可：拉高触发一次，拉低重新武装；运行中拉低则中止。
 * uint8_t 在当前 Cortex-M7 上单次读写是原子的，volatile 保证任务每次都重新读取。
 */
volatile uint8_t arm_lift_enable = 0U;

#define ACTION_REQUEST_QUEUE_DEPTH 4U
#define ACTION_RESULT_QUEUE_DEPTH 4U

typedef enum {
    ACTION_TASK_REQUEST_START = 0,
    ACTION_TASK_REQUEST_RESET,
} ActionTaskRequestType;

typedef struct {
    ActionTaskRequestType type;
    ActionTarget target;
    ActionGroup group;
} ActionTaskRequest;

typedef enum {
    ACTION_TASK_RESULT_STARTED = 0,
    ACTION_TASK_RESULT_REJECTED,
    ACTION_TASK_RESULT_FINISHED,
    ACTION_TASK_RESULT_RESET_DONE,
} ActionTaskResultType;

typedef struct {
    ActionTaskResultType type;
    ActionTarget target;
    ActionGroup group;
} ActionTaskResult;

typedef struct {
    bool active;
    ActionGroup group;
} ActionTaskExecution;

typedef enum {
    ARM_ACTION_STATE_WAIT_ENABLE = 0,
    ARM_ACTION_STATE_WAIT_START_RESULT,
    ARM_ACTION_STATE_RUNNING,
    ARM_ACTION_STATE_WAIT_RESET_RESULT,
    ARM_ACTION_STATE_WAIT_RELEASE,
} ArmActionState;

static osMessageQueueId_t action_request_queue;
static osMessageQueueId_t action_result_queue;
static ActionTaskExecution action_executions[ACTION_TARGET_COUNT];

bool TaskAction_Init(void) {
    if (action_request_queue != NULL || action_result_queue != NULL) {
        return action_request_queue != NULL && action_result_queue != NULL;
    }

    action_request_queue = osMessageQueueNew(ACTION_REQUEST_QUEUE_DEPTH,
                                             sizeof(ActionTaskRequest),
                                             NULL);
    action_result_queue = osMessageQueueNew(ACTION_RESULT_QUEUE_DEPTH,
                                            sizeof(ActionTaskResult),
                                            NULL);

    return action_request_queue != NULL && action_result_queue != NULL;
}

static bool ArmTask_SendActionRequest(ActionTaskRequestType type,
                                      ActionTarget target,
                                      ActionGroup group) {
    ActionTaskRequest request;

    if (action_request_queue == NULL)
        return false;

    request.type = type;
    request.target = target;
    request.group = group;
    return osMessageQueuePut(action_request_queue, &request, 0U, 0U) == osOK;
}

static void TaskAction_PublishResult(ActionTaskResultType type,
                                     ActionTarget target,
                                     ActionGroup group) {
    ActionTaskResult result;

    if (action_result_queue == NULL)
        return;

    result.type = type;
    result.target = target;
    result.group = group;
    (void)osMessageQueuePut(action_result_queue, &result, 0U, 0U);
}

/**
 * @brief 处理上层动作请求。
 * @note Action_Start/Reset 只在 TaskAction 上下文调用，避免与 Action_Update 并发修改队列。
 */
static void TaskAction_ProcessRequests(void) {
    ActionTaskRequest request;

    if (action_request_queue == NULL)
        return;

    while (osMessageQueueGet(action_request_queue, &request, NULL, 0U) == osOK) {
        if (request.target >= ACTION_TARGET_COUNT) {
            TaskAction_PublishResult(ACTION_TASK_RESULT_REJECTED,
                                     request.target,
                                     request.group);
            continue;
        }

        switch (request.type) {
            case ACTION_TASK_REQUEST_START: {
                ActionId actions[] = {request.group, ACTION_GROUP_NULL};
                ActionTaskExecution* execution = &action_executions[request.target];

                if (execution->active ||
                    Action_GroupState(request.target) != ACTION_STATE_IDLE ||
                    !Action_Start(request.target, actions)) {
                    TaskAction_PublishResult(ACTION_TASK_RESULT_REJECTED,
                                             request.target,
                                             request.group);
                    break;
                }

                execution->active = true;
                execution->group = request.group;
                TaskAction_PublishResult(ACTION_TASK_RESULT_STARTED,
                                         request.target,
                                         request.group);
                break;
            }

            case ACTION_TASK_REQUEST_RESET: {
                ActionTaskExecution* execution = &action_executions[request.target];
                ActionGroup active_group = execution->active ? execution->group
                                                              : request.group;

                Action_Reset(request.target);
                execution->active = false;
                execution->group = ACTION_GROUP_NULL;
                TaskAction_PublishResult(ACTION_TASK_RESULT_RESET_DONE,
                                         request.target,
                                         active_group);
                break;
            }

            default:
                TaskAction_PublishResult(ACTION_TASK_RESULT_REJECTED,
                                         request.target,
                                         request.group);
                break;
        }
    }
}

/** @brief 将已启动队列进入 IDLE 的变化反馈给上层业务状态机。 */
static void TaskAction_ReportFinishedActions(void) {
    uint8_t target;

    for (target = 0U; target < ACTION_TARGET_COUNT; target++) {
        ActionTaskExecution* execution = &action_executions[target];

        if (!execution->active ||
            Action_GroupState((ActionTarget)target) != ACTION_STATE_IDLE) {
            continue;
        }

        TaskAction_PublishResult(ACTION_TASK_RESULT_FINISHED,
                                 (ActionTarget)target,
                                 execution->group);
        execution->active = false;
        execution->group = ACTION_GROUP_NULL;
    }
}

void MotorTask(void* argument) {
    (void)argument;
    MYACTUATOR_TestCommand myactuator_command;
    UNITREE_A1_TestCommand unitree_command;
    DAMIAO_TestCommand damiao_command;
    ROBSTRIDE_TestCommand robstride_command;
    bool myactuator_command_valid;
    bool unitree_command_valid;
    bool damiao_command_valid;
    bool robstride_command_valid;
    uint32_t wake_tick;

    wake_tick = osKernelGetTickCount();

    for (;;) {
        /*
         * 发布进行中或读取期间版本发生变化时，本周期撤销使能；如果此前已
         * 使能则发送一次停止命令。下一周期再次尝试读取，避免消费半更新数据。
         */
        myactuator_command_valid = MYACTUATOR_Comm_TryReadCommand(&myactuator_command);
        unitree_command_valid = MotorComm_TryReadUnitreeCommand(&unitree_command);
        damiao_command_valid = DamiaoComm_TryReadCommand(&damiao_command);
        robstride_command_valid = RobStrideComm_TryReadCommand(&robstride_command);

        if (FDCAN_Service(&can1) == FDCAN_ERR_NONE) {
            if (myactuator_command_valid) {
                MYACTUATOR_Comm_ApplyCommand(&myactuator_command);
            } else {
                MYACTUATOR_Comm_ApplyCommand(NULL);
            }

            if (damiao_command_valid) {
                DamiaoComm_ApplyCommand(&damiao_command);
            } else {
                DamiaoComm_ApplyCommand(NULL);
            }

            if (robstride_command_valid) {
                RobStrideComm_ApplyCommand(&robstride_command);
            } else {
                RobStrideComm_ApplyCommand(NULL);
            }

            MYACTUATOR_Comm_ProcessAndTransmit();
            DamiaoComm_ProcessAndTransmit();
            RobStrideComm_ProcessAndTransmit();
        }

        /* A1 使用独立的 USART1 链路，不受 CAN 总线恢复状态影响。 */
        if (unitree_command_valid) {
            MotorComm_ApplyUnitreeCommand(&unitree_command);
        } else {
            MotorComm_ApplyUnitreeCommand(NULL);
        }
        MotorComm_ProcessAndTransmit();

        /* 当前 FreeRTOS tick 为 1 ms，超期后重新对齐，避免突发补发。 */
        wake_tick += 1U;
        if (osDelayUntil(wake_tick) != osOK) {
            wake_tick = osKernelGetTickCount();
            (void)osDelayUntil(++wake_tick);
        }
    }
}

/**
 * @brief 机械臂上层控制任务
 * @note 业务状态机只发送动作请求、消费调度结果，不直接调用动作库 API。
 */
void ArmTask(void* argument) {
    ArmActionState lift_state = ARM_ACTION_STATE_WAIT_ENABLE;
    ActionTaskResult result;

    (void)argument;

    for (;;) {
        /* 先消费 TaskAction 的结果，再根据本周期许可状态决定下一步。 */
        while (action_result_queue != NULL &&
               osMessageQueueGet(action_result_queue, &result, NULL, 0U) == osOK) {
            if (result.target != ACTION_TARGET_ARM ||
                result.group != ACTION_GROUP_ARM_LIFT) {
                continue;
            }

            switch (result.type) {
                case ACTION_TASK_RESULT_STARTED:
                    if (lift_state == ARM_ACTION_STATE_WAIT_START_RESULT) {
                        lift_state = ARM_ACTION_STATE_RUNNING;
                    }
                    break;

                case ACTION_TASK_RESULT_REJECTED:
                    if (lift_state == ARM_ACTION_STATE_WAIT_START_RESULT) {
                        /* 初始化时序未就绪时保持自动重试，不要求重新翻转许可位。 */
                        lift_state = ARM_ACTION_STATE_WAIT_ENABLE;
                    }
                    break;

                case ACTION_TASK_RESULT_FINISHED:
                    if (lift_state == ARM_ACTION_STATE_RUNNING)
                        lift_state = ARM_ACTION_STATE_WAIT_RELEASE;
                    break;

                case ACTION_TASK_RESULT_RESET_DONE:
                    if (lift_state == ARM_ACTION_STATE_WAIT_RESET_RESULT) {
                        lift_state = (arm_lift_enable != 0U)
                                         ? ARM_ACTION_STATE_WAIT_RELEASE
                                         : ARM_ACTION_STATE_WAIT_ENABLE;
                    }
                    break;

                default:
                    break;
            }
        }

        switch (lift_state) {
            case ARM_ACTION_STATE_WAIT_ENABLE:
                if (arm_lift_enable != 0U &&
                    ArmTask_SendActionRequest(ACTION_TASK_REQUEST_START,
                                              ACTION_TARGET_ARM,
                                              ACTION_GROUP_ARM_LIFT)) {
                    lift_state = ARM_ACTION_STATE_WAIT_START_RESULT;
                }
                break;

            case ARM_ACTION_STATE_RUNNING:
                if (arm_lift_enable == 0U &&
                    ArmTask_SendActionRequest(ACTION_TASK_REQUEST_RESET,
                                              ACTION_TARGET_ARM,
                                              ACTION_GROUP_ARM_LIFT)) {
                    lift_state = ARM_ACTION_STATE_WAIT_RESET_RESULT;
                }
                break;

            case ARM_ACTION_STATE_WAIT_RELEASE:
                /* 动作完成后仍保持目标；许可清零时必须显式发送两轴失能帧。 */
                if (arm_lift_enable == 0U &&
                    ArmTask_SendActionRequest(ACTION_TASK_REQUEST_RESET,
                                              ACTION_TARGET_ARM,
                                              ACTION_GROUP_ARM_LIFT)) {
                    lift_state = ARM_ACTION_STATE_WAIT_RESET_RESULT;
                }
                break;

            case ARM_ACTION_STATE_WAIT_START_RESULT:
                /* START 尚在队列中时撤销许可，也必须跟进 RESET/失能请求。 */
                if (arm_lift_enable == 0U &&
                    ArmTask_SendActionRequest(ACTION_TASK_REQUEST_RESET,
                                              ACTION_TARGET_ARM,
                                              ACTION_GROUP_ARM_LIFT)) {
                    lift_state = ARM_ACTION_STATE_WAIT_RESET_RESULT;
                }
                break;

            case ARM_ACTION_STATE_WAIT_RESET_RESULT:
            default:
                break;
        }

        osDelay(1U);
    }
}

/**
 * @brief 动作队列任务
 *
 * 沿用 ArmPlatform_H7 的实现方式，将 Action_Update 放在独立任务中以 1 ms
 * 周期推进。使用 osDelayUntil 保持固定节拍，避免动作步骤的 tick 延时随任务
 * 执行时间逐渐漂移。
 */
void TaskAction(void* argument) {
    (void)argument;
    uint32_t wake_tick;
    wake_tick = osKernelGetTickCount();

    for (;;) {
        TaskAction_ProcessRequests();
        Action_Update();
        TaskAction_ReportFinishedActions();

        wake_tick += 1U;
        if (osDelayUntil(wake_tick) != osOK) {
            wake_tick = osKernelGetTickCount();
            (void)osDelayUntil(++wake_tick);
        }
    }
}
