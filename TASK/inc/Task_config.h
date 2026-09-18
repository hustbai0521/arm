#ifndef TASK_CONFIG_H_
#define TASK_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LIFT 动作触发许可。
 * @note 置 1 后触发一次 LIFT；动作完成后必须先清 0，下一次置 1 才会再次触发。
 *       动作运行期间清 0 会请求 TaskAction 复位动作并安全停止电机。
 */
extern volatile uint8_t arm_lift_enable;

/**
 * @brief 创建 ArmTask 与 TaskAction 之间的动作请求/结果消息队列。
 * @note 必须在 osKernelInitialize 之后、创建这两个任务之前调用一次。
 */
bool TaskAction_Init(void);

/** 每 1 ms 调度各电机 App 通信端口，并处理 CAN 总线恢复。 */
void MotorTask(void* argument);

/** 机械臂上层控制任务；负责业务状态机和动作请求。 */
void ArmTask(void* argument);

/** 动作调度任务；独占动作库 API，并每 1 ms 推进动作队列。 */
void TaskAction(void* argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_CONFIG_H_ */
