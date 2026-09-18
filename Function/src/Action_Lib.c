/**
 * @file    Action_Lib.c
 * @brief   Generic action scheduler library implementation
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-30
 */

#include "Action_Lib.h"
#include "cmsis_os.h"
#include <stddef.h>
#include <string.h>

typedef struct
{
    const char* tag;
    ActionQueue* owner;
} ActionGlobalTagLock;

typedef struct
{
    const char* tag;
    const ActionDeviceInstance* instance;
    ActionQueue* owner;
} ActionLocalTagLock;

/// 全局资源标签锁表，由所有动作队列共享
static ActionGlobalTagLock action_global_tag_locks[ACTION_TAG_LOCK_MAX] = {0};

/// 实例内标签锁表，仅在同一个实例被多个队列共享时生效
static ActionLocalTagLock action_local_tag_locks[ACTION_TAG_LOCK_MAX] = {0};

typedef enum
{
    ACTION_TAG_SCOPE_GLOBAL = 0,
    ACTION_TAG_SCOPE_LOCAL,
} ActionTagScope;

typedef enum
{
    ACTION_STEP_STATE_CMD = 0U,
    ACTION_STEP_STATE_WAIT,
    ACTION_STEP_STATE_DELAY,
} ActionStepSubState;

/* ==========================================================================
 * Internal helpers
 * ========================================================================== */

static ActionExecHandle* Action_FindHandle(ActionExecId id) {
    uint8_t handle_count;

    if (Action_Runtime.exec_handles == NULL)
        return NULL;

    handle_count = (Action_Runtime.exec_handle_count <= ACTION_EXEC_HANDLE_MAX) ? Action_Runtime.exec_handle_count
                                                                                : ACTION_EXEC_HANDLE_MAX;

    for (uint8_t i = 0U; i < handle_count; i++) {
        ActionExecHandle* handle = &Action_Runtime.exec_handles[i];

        if (handle->id == id)
            return handle;
    }

    return NULL;
}

static const ActionGroupDef* Action_FindGroupDef(const ActionDevice* device, ActionId id) {
    if (device == NULL || device->entries == NULL)
        return NULL;

    for (uint16_t i = 0U; i < device->entry_count; i++) {
        if (device->entries[i].id == id)
            return &device->entries[i].group;
    }

    return NULL;
}

static bool Action_InstanceReady(const ActionDeviceInstance* instance) {
    const ActionDevice* device;

    if (instance == NULL || instance->device_class == NULL)
        return false;

    device = instance->device_class;
    return device->is_available == NULL || device->is_available(instance->ctx);
}

static void Action_ResetInstance(const ActionDeviceInstance* instance) {
    if (instance != NULL && instance->device_class != NULL && instance->device_class->reset != NULL)
        instance->device_class->reset(instance->ctx);
}

static void Action_ReleaseLocks(ActionQueue* queue) {
    if (queue == NULL)
        return;

    for (uint8_t i = 0U; i < ACTION_TAG_LOCK_MAX; i++) {
        if (action_global_tag_locks[i].owner == queue) {
            action_global_tag_locks[i].tag = NULL;
            action_global_tag_locks[i].owner = NULL;
        }

        if (action_local_tag_locks[i].owner == queue) {
            action_local_tag_locks[i].tag = NULL;
            action_local_tag_locks[i].instance = NULL;
            action_local_tag_locks[i].owner = NULL;
        }
    }
}

static void Action_ClearQueue(ActionQueue* queue) {
    if (queue == NULL)
        return;

    for (uint16_t i = 0U; i < ACTION_GROUP_QUEUE_MAX; i++)
        queue->group_queue[i] = NULL;

    memset(queue->user_floats, 0, sizeof(queue->user_floats));

    queue->GroupState = ACTION_STATE_IDLE;
    queue->ActState = ACTION_STATE_IDLE;
    queue->current_group = NULL;
    queue->group_idx = 0U;
    queue->step_queue = NULL;
    queue->step_count = 0U;
    queue->step_idx = 0U;
    queue->sub_state = ACTION_STEP_STATE_CMD;
    queue->start_tick = 0U;
    queue->waiting = false;
    queue->instance = NULL;
}

static void Action_StopQueue(ActionQueue* queue) {
    if (queue == NULL)
        return;

    Action_ReleaseLocks(queue);
    Action_ClearQueue(queue);
    queue->open_wait = true;
}

static bool Action_LocksFree(ActionTagScope scope, ActionQueue* queue, const ActionDeviceInstance* instance,
                             const char* const* tags, uint8_t count) {
    for (uint8_t i = 0U; i < count; i++) {
        for (uint8_t lock_idx = 0U; lock_idx < ACTION_TAG_LOCK_MAX; lock_idx++) {
            if (scope == ACTION_TAG_SCOPE_GLOBAL) {
                if (action_global_tag_locks[lock_idx].owner != NULL &&
                    action_global_tag_locks[lock_idx].owner != queue &&
                    Action_StrEq(action_global_tag_locks[lock_idx].tag, tags[i]))
                    return false;
            } else if (action_local_tag_locks[lock_idx].instance == instance &&
                       action_local_tag_locks[lock_idx].owner != NULL &&
                       action_local_tag_locks[lock_idx].owner != queue &&
                       Action_StrEq(action_local_tag_locks[lock_idx].tag, tags[i])) {
                return false;
            }
        }
    }

    return true;
}

static bool Action_LockTagsImpl(ActionTagScope scope, ActionQueue* queue, const ActionDeviceInstance* instance,
                                const char* const* tags, uint8_t count) {
    for (uint8_t i = 0U; i < count; i++) {
        bool locked = false;

        for (uint8_t lock_idx = 0U; lock_idx < ACTION_TAG_LOCK_MAX; lock_idx++) {
            if (scope == ACTION_TAG_SCOPE_GLOBAL) {
                if (action_global_tag_locks[lock_idx].owner == queue &&
                    Action_StrEq(action_global_tag_locks[lock_idx].tag, tags[i])) {
                    locked = true;
                    break;
                }
            } else if (action_local_tag_locks[lock_idx].owner == queue &&
                       action_local_tag_locks[lock_idx].instance == instance &&
                       Action_StrEq(action_local_tag_locks[lock_idx].tag, tags[i])) {
                locked = true;
                break;
            }
        }

        if (locked)
            continue;

        for (uint8_t lock_idx = 0U; lock_idx < ACTION_TAG_LOCK_MAX; lock_idx++) {
            if (scope == ACTION_TAG_SCOPE_GLOBAL) {
                if (action_global_tag_locks[lock_idx].owner == NULL) {
                    action_global_tag_locks[lock_idx].tag = tags[i];
                    action_global_tag_locks[lock_idx].owner = queue;
                    locked = true;
                    break;
                }
            } else if (action_local_tag_locks[lock_idx].owner == NULL) {
                action_local_tag_locks[lock_idx].tag = tags[i];
                action_local_tag_locks[lock_idx].instance = instance;
                action_local_tag_locks[lock_idx].owner = queue;
                locked = true;
                break;
            }
        }

        if (!locked) {
            Action_ReleaseLocks(queue);
            return false;
        }
    }

    return true;
}

static bool Action_CollectTags(const ActionGroupDef* const* groups, const char** tags, uint8_t* count,
                               ActionTagScope scope) {
    if (groups == NULL || tags == NULL || count == NULL)
        return false;

    *count = 0U;

    for (uint16_t i = 0U; i < ACTION_GROUP_QUEUE_MAX; i++) {
        const ActionGroupDef* group = groups[i];
        const char* const* group_tags;
        uint8_t group_tag_count;

        if (group == NULL)
            return true;
        if (group->steps == NULL && group->step_count == 0U)
            continue;

        if (scope == ACTION_TAG_SCOPE_GLOBAL) {
            group_tags = group->global_tags;
            group_tag_count = group->global_tag_count;
        } else {
            group_tags = group->local_tags;
            group_tag_count = group->local_tag_count;
        }

        if (group_tags == NULL || group_tag_count == 0U)
            continue;

        for (uint8_t tag_idx = 0U; tag_idx < group_tag_count; tag_idx++) {
            const char* tag = group_tags[tag_idx];
            bool duplicate = false;

            for (uint8_t existing_idx = 0U; existing_idx < *count; existing_idx++) {
                if (Action_StrEq(tags[existing_idx], tag)) {
                    duplicate = true;
                    break;
                }
            }

            if (tag == NULL || duplicate)
                continue;
            if (*count >= ACTION_TAG_REQUEST_MAX)
                return false;

            tags[*count] = tag;
            (*count)++;
        }
    }

    return true;
}

static uint8_t Action_StepItemCount(const void* items, uint8_t explicit_count, uint8_t max_count, size_t item_size,
                                    size_t op_offset) {
    const uint8_t* item_ptr = (const uint8_t*)items;

    if (item_ptr == NULL)
        return 0U;
    if (explicit_count > 0U && explicit_count <= max_count)
        return explicit_count;

    for (uint8_t i = 0U; i < max_count; i++) {
        const char* const* op = (const char* const*)(item_ptr + ((size_t)i * item_size) + op_offset);

        if (*op == NULL)
            return i;
    }

    return max_count;
}

static uint8_t Action_StepCommandCount(const ActionStep* step) {
    if (step == NULL)
        return 0U;

    return Action_StepItemCount(step->commands, step->command_count, ACTION_STEP_MAX_COMMANDS, sizeof(step->commands[0]),
                                offsetof(ActionCommand, op));
}

static uint8_t Action_StepWaitCount(const ActionStep* step) {
    if (step == NULL)
        return 0U;

    return Action_StepItemCount(step->waits, step->wait_count, ACTION_STEP_MAX_WAITS, sizeof(step->waits[0]),
                                offsetof(ActionWaitCondition, op));
}

static bool Action_RunCommands(ActionQueue* queue, const ActionStep* step) {
    const ActionDeviceInstance* instance;
    const ActionDevice* device;
    uint8_t count;

    if (queue == NULL || step == NULL)
        return true;

    instance = queue->instance;
    device = (instance != NULL) ? instance->device_class : NULL;
    if (device == NULL || !Action_InstanceReady(instance)) {
        Action_StopQueue(queue);
        return false;
    }

    count = Action_StepCommandCount(step);

    for (uint8_t i = 0U; i < count; i++) {
        const ActionCommand* cmd = &step->commands[i];

        if (device->exec == NULL || !device->exec(instance->ctx, queue, cmd)) {
            Action_StopQueue(queue);
            return false;
        }
    }

    return true;
}

static bool Action_WaitsReady(ActionQueue* queue, const ActionStep* step) {
    const ActionDeviceInstance* instance;
    const ActionDevice* device;
    uint8_t count;

    if (queue == NULL || step == NULL)
        return true;

    instance = queue->instance;
    device = (instance != NULL) ? instance->device_class : NULL;
    if (device == NULL || !Action_InstanceReady(instance)) {
        Action_StopQueue(queue);
        return false;
    }

    count = Action_StepWaitCount(step);

    for (uint8_t i = 0U; i < count; i++) {
        const ActionWaitCondition* wait = &step->waits[i];

        if (device->is_done == NULL || !device->is_done(instance->ctx, queue, wait))
            return false;
    }

    return true;
}

static bool Action_AdvanceStep(ActionQueue* queue, const ActionStep* step) {
    if (queue == NULL || step == NULL)
        return false;

    switch (queue->sub_state) {
        case ACTION_STEP_STATE_CMD:
            if (!Action_RunCommands(queue, step))
                return false;
            if (queue->GroupState != ACTION_STATE_EXECUTING)
                return false;
            queue->sub_state = ACTION_STEP_STATE_WAIT;
            break;

        case ACTION_STEP_STATE_WAIT:
            if (Action_WaitsReady(queue, step)) {
                queue->start_tick = osKernelGetTickCount();
                queue->sub_state = ACTION_STEP_STATE_DELAY;
            }
            break;

        case ACTION_STEP_STATE_DELAY:
            if ((osKernelGetTickCount() - queue->start_tick) >= step->dly)
                return true;
            break;

        default:
            queue->sub_state = ACTION_STEP_STATE_CMD;
            break;
    }

    return false;
}

static bool Action_LoadNextGroup(ActionQueue* queue) {
    while (queue->step_queue == NULL) {
        const ActionGroupDef* group;

        if (queue->group_idx == 0U)
            queue->group_idx = 1U;
        else
            queue->group_idx++;

        if (queue->group_idx > ACTION_GROUP_QUEUE_MAX)
            return false;

        group = queue->group_queue[queue->group_idx - 1U];
        if (group == NULL)
            return false;

        queue->step_queue = group->steps;
        queue->step_count = group->step_count;
        queue->current_group = group;
        queue->step_idx = 1U;
        queue->sub_state = ACTION_STEP_STATE_CMD;
        memset(queue->user_floats, 0, sizeof(queue->user_floats));

        if (queue->step_count == 0U) {
            queue->current_group = NULL;
            queue->step_queue = NULL;
            queue->step_count = 0U;
            queue->step_idx = 0U;
            continue;
        }

        return true;
    }

    return true;
}

static void Action_SetQueueIdle(ActionQueue* queue, bool release_locks) {
    if (queue == NULL)
        return;

    if (release_locks)
        Action_ReleaseLocks(queue);

    queue->GroupState = ACTION_STATE_IDLE;
    queue->ActState = ACTION_STATE_IDLE;
    queue->waiting = false;
}

static void Action_UpdateQueue(ActionQueue* queue) {
    if (queue == NULL || queue->GroupState != ACTION_STATE_EXECUTING)
        return;

    if (!Action_InstanceReady(queue->instance)) {
        Action_StopQueue(queue);
        return;
    }

    if (queue->waiting)
        return;

    queue->ActState = ACTION_STATE_EXECUTING;

    if (queue->step_queue == NULL) {
        if (!Action_LoadNextGroup(queue)) {
            Action_SetQueueIdle(queue, true);
            return;
        }
    }

    if (queue->step_idx == 0U || queue->step_idx > queue->step_count) {
        Action_SetQueueIdle(queue, true);
        return;
    }

    if (!Action_AdvanceStep(queue, &queue->step_queue[queue->step_idx - 1U]))
        return;

    queue->sub_state = ACTION_STEP_STATE_CMD;

    if (queue->step_idx < queue->step_count) {
        queue->step_idx++;
        return;
    }

    if (queue->current_group != NULL && queue->current_group->on_finish != NULL)
        queue->current_group->on_finish(queue->current_group->user, queue);

    queue->current_group = NULL;
    queue->step_queue = NULL;
    queue->step_count = 0U;
    queue->step_idx = 0U;

    if (queue->group_idx < ACTION_GROUP_QUEUE_MAX && queue->group_queue[queue->group_idx] != NULL) {
        if (queue->open_wait)
            queue->waiting = true;
        queue->ActState = ACTION_STATE_IDLE;
        return;
    }

    Action_SetQueueIdle(queue, true);
}

static bool Action_StartQueue(ActionQueue* queue, const ActionId* ids, const ActionDeviceInstance* instance) {
    const ActionDevice* device = (instance != NULL) ? instance->device_class : NULL;
    const ActionGroupDef* groups[ACTION_GROUP_QUEUE_MAX];
    const char* global_tags[ACTION_TAG_REQUEST_MAX] = {0};
    const char* local_tags[ACTION_TAG_REQUEST_MAX] = {0};
    uint8_t global_tag_count = 0U;
    uint8_t local_tag_count = 0U;
    bool has_group = false;

    if (queue == NULL || ids == NULL || device == NULL)
        return false;
    if (queue->GroupState != ACTION_STATE_IDLE)
        return false;
    if (!Action_InstanceReady(instance))
        return false;

    for (uint16_t i = 0U; i < ACTION_GROUP_QUEUE_MAX; i++) {
        groups[i] = NULL;

        if (ids[i] == device->null_id)
            break;

        groups[i] = Action_FindGroupDef(device, ids[i]);
        if (groups[i] == NULL)
            return false;

        has_group = true;
        if (i == (ACTION_GROUP_QUEUE_MAX - 1U))
            return false;
    }

    if (!has_group || !Action_CollectTags(groups, global_tags, &global_tag_count, ACTION_TAG_SCOPE_GLOBAL) ||
        !Action_CollectTags(groups, local_tags, &local_tag_count, ACTION_TAG_SCOPE_LOCAL) ||
        !Action_LocksFree(ACTION_TAG_SCOPE_GLOBAL, queue, NULL, global_tags, global_tag_count) ||
        !Action_LockTagsImpl(ACTION_TAG_SCOPE_GLOBAL, queue, NULL, global_tags, global_tag_count) ||
        !Action_LocksFree(ACTION_TAG_SCOPE_LOCAL, queue, instance, local_tags, local_tag_count) ||
        !Action_LockTagsImpl(ACTION_TAG_SCOPE_LOCAL, queue, instance, local_tags, local_tag_count)) {
        Action_ReleaseLocks(queue);
        return false;
    }

    memcpy(queue->group_queue, groups, sizeof(queue->group_queue));
    memset(queue->user_floats, 0, sizeof(queue->user_floats));

    queue->group_idx = 0U;
    queue->current_group = NULL;
    queue->step_queue = NULL;
    queue->step_count = 0U;
    queue->step_idx = 0U;
    queue->sub_state = ACTION_STEP_STATE_CMD;
    queue->start_tick = 0U;
    queue->GroupState = ACTION_STATE_EXECUTING;
    queue->ActState = ACTION_STATE_IDLE;
    queue->waiting = false;
    queue->instance = instance;

    return true;
}

bool Action_Start(ActionExecId id, const ActionId* ids) {
    ActionExecHandle* handle = Action_FindHandle(id);

    if (handle == NULL || ids == NULL || handle->instance == NULL)
        return false;

    return Action_StartQueue(&handle->queue, ids, handle->instance);
}

bool Action_Skip(ActionExecId id) {
    ActionExecHandle* handle = Action_FindHandle(id);

    if (handle == NULL || !handle->queue.waiting)
        return false;

    handle->queue.waiting = false;
    return true;
}

void Action_Reset(ActionExecId id) {
    ActionExecHandle* handle = Action_FindHandle(id);

    if (handle == NULL)
        return;

    Action_StopQueue(&handle->queue);
    Action_ResetInstance(handle->instance);
}

ActionState Action_GroupState(ActionExecId id) {
    ActionExecHandle* handle = Action_FindHandle(id);
    return (handle != NULL) ? handle->queue.GroupState : ACTION_STATE_IDLE;
}

ActionState Action_ActState(ActionExecId id) {
    ActionExecHandle* handle = Action_FindHandle(id);
    return (handle != NULL) ? handle->queue.ActState : ACTION_STATE_IDLE;
}

void Action_ResetAll(void) {
    uint8_t handle_count;

    if (Action_Runtime.exec_handles == NULL)
        return;

    handle_count = (Action_Runtime.exec_handle_count <= ACTION_EXEC_HANDLE_MAX) ? Action_Runtime.exec_handle_count
                                                                                : ACTION_EXEC_HANDLE_MAX;

    for (uint8_t i = 0U; i < handle_count; i++) {
        ActionExecHandle* handle = &Action_Runtime.exec_handles[i];

        Action_StopQueue(&handle->queue);
        Action_ResetInstance(handle->instance);
    }
}

void Action_Update(void) {
    uint8_t handle_count;

    if (Action_Runtime.exec_handles == NULL)
        return;

    handle_count = (Action_Runtime.exec_handle_count <= ACTION_EXEC_HANDLE_MAX) ? Action_Runtime.exec_handle_count
                                                                                : ACTION_EXEC_HANDLE_MAX;

    for (uint8_t i = 0U; i < handle_count; i++)
        Action_UpdateQueue(&Action_Runtime.exec_handles[i].queue);
}

bool Action_StrEq(const char* a, const char* b) {
    if (a == NULL || b == NULL)
        return false;

    return strcmp(a, b) == 0;
}
