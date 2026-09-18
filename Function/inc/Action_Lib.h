/**
 * @file    Action_Lib.h
 * @brief   Generic action scheduler library interface
 * @author  Zhexuan Xu 15779212418@qq.com
 * @date    2026-05-30
 *
 * This library only owns queue scheduling, step execution, delay timing and
 * optional resource locks. Device behavior and action catalogs are provided by
 * application-defined device classes and device instances.
 */

#ifndef ACTION_LIB_H
#define ACTION_LIB_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 单步最大命令数 */
#define ACTION_STEP_MAX_COMMANDS 4U

/** @brief 单步最大等待条件数 */
#define ACTION_STEP_MAX_WAITS 4U

/** @brief 队列最大动作组数 */
#define ACTION_GROUP_QUEUE_MAX 32U

/** @brief 标签锁表最大槽位数 */
#define ACTION_TAG_LOCK_MAX 16U

/** @brief 单次启动最大标签数 */
#define ACTION_TAG_REQUEST_MAX 16U

/** @brief 最大执行实例数 */
#define ACTION_EXEC_HANDLE_MAX 8U

/** @brief 队列私有浮点槽位数 */
#define ACTION_QUEUE_USER_FLOAT_COUNT 4U
typedef uint16_t ActionId;        ///< 应用层定义的动作组 ID
typedef uint16_t ActionExecId;    ///< 应用层定义的动作运行实例 ID
typedef uint8_t ActionCmdTarget;  ///< 应用层定义的实例内端口/子通道 ID，仅用于设备类内部路由

/**
 * @brief 动作参数负载类型
 */
typedef enum {
    ACTION_VALUE_NONE = 0,    ///< 无负载
    ACTION_VALUE_BOOL,        ///< 布尔值负载
    ACTION_VALUE_FLOAT,       ///< 单浮点负载
    ACTION_VALUE_PAIR_FLOAT,  ///< 双浮点负载
    ACTION_VALUE_NAMED_BOOL,  ///< 带名称的布尔值负载
    ACTION_VALUE_CUSTOM,      ///< 应用层自定义负载指针
} ActionValueType;

/**
 * @brief 动作队列运行状态
 */
typedef enum {
    ACTION_STATE_IDLE = 1,       ///< 队列或步骤空闲
    ACTION_STATE_EXECUTING = 0,  ///< 队列或步骤执行中
} ActionState;

/**
 * @brief 双浮点参数
 */
typedef struct
{
    float a;  ///< 第一个值
    float b;  ///< 第二个值
} ActionPairFloat;

/**
 * @brief 带名称的布尔值
 */
typedef struct
{
    const char* name;  ///< 应用层定义的名称字符串
    bool value;        ///< 布尔值
} ActionNamedBool;

/**
 * @brief 自定义负载引用
 */
typedef struct
{
    const void* ptr;  ///< 负载地址
    uint16_t size;    ///< 负载大小 (字节)
} ActionCustomValue;

/**
 * @brief 通用命令或等待参数
 */
typedef struct
{
    ActionValueType type;  ///< 负载类型，决定读取 data 中的哪个成员
    union {
        bool b;                      ///< 布尔值负载
        float f;                     ///< 浮点负载
        ActionPairFloat pair;        ///< 双浮点负载
        ActionNamedBool named_bool;  ///< 带名称的布尔值负载
        ActionCustomValue custom;    ///< 自定义负载引用
    } data;                          ///< 负载数据
} ActionValue;

/**
 * @brief 分发给目标设备实例的通用命令
 */
typedef struct
{
    ActionCmdTarget target;  ///< 当前实例内部端口
    const char* op;          ///< 操作名
    ActionValue value;       ///< 操作参数
} ActionCommand;

/**
 * @brief 由目标设备实例检测的通用等待条件
 */
typedef struct
{
    ActionCmdTarget target;  ///< 当前实例内部端口
    const char* op;          ///< 等待操作名
    ActionValue expected;    ///< 期望值
} ActionWaitCondition;

/**
 * @brief 一个动作步骤，包含命令、等待条件和延时
 */
typedef struct
{
    ActionCommand commands[ACTION_STEP_MAX_COMMANDS];  ///< 同阶段下发的命令
    uint8_t command_count;                             ///< 有效命令数量
    ActionWaitCondition waits[ACTION_STEP_MAX_WAITS];  ///< 等待条件列表
    uint8_t wait_count;                                ///< 有效等待条件数量
    uint32_t dly;                                      ///< 等待满足后的延时 (tick)
} ActionStep;

typedef struct ActionGroupDef ActionGroupDef;
typedef struct ActionDevice ActionDevice;

/** @brief 设备实例 */
typedef struct
{
    const ActionDevice* device_class;  ///< 设备实例所属类型
    void* ctx;                         ///< 设备实例上下文
} ActionDeviceInstance;

/** @brief 动作队列运行时状态 */
typedef struct
{
    /* ---- 队列数据 ---- */
    const ActionGroupDef* group_queue[ACTION_GROUP_QUEUE_MAX];  ///< 已注入的动作组定义
    const ActionGroupDef* current_group;                        ///< 当前动作组定义
    const ActionStep* step_queue;                               ///< 当前步骤数组
    const ActionDeviceInstance* instance;                       ///< 当前队列绑定的设备实例

    /* ---- 计数与时间 ---- */
    uint16_t group_idx;                                         ///< 当前动作组索引 (1-based, Action_Start 装载新队列时清 0)
    uint16_t step_count;                                        ///< 当前动作组步骤数量
    uint16_t step_idx;                                          ///< 当前步骤索引 (1-based, 0 表示无 active step)
    uint32_t start_tick;                                        ///< 延时起始 tick

    /* ---- 状态标志 ---- */
    ActionState GroupState;                                     ///< 整个队列状态
    ActionState ActState;                                       ///< 当前步骤状态
    uint8_t sub_state;                                          ///< 步骤状态机阶段
    bool open_wait;                                             ///< 是否允许动作组间暂停
    bool waiting;                                               ///< 队列正在等待外部放行

    /* ---- 队列私有存储 ---- */
    float user_floats[ACTION_QUEUE_USER_FLOAT_COUNT];           ///< 队列私有浮点存储
} ActionQueue;

/**
 * @brief 静态动作组定义
 */
struct ActionGroupDef {
    const ActionStep* steps;                            ///< 步骤数组
    uint16_t step_count;                                ///< 步骤数量
    const char* const* global_tags;                     ///< 全局跨队列资源标签列表
    uint8_t global_tag_count;                           ///< 全局资源标签数量
    const char* const* local_tags;                      ///< 同实例内资源标签列表
    uint8_t local_tag_count;                            ///< 同实例内资源标签数量
    void (*on_finish)(void* user, ActionQueue* queue);  ///< 可选动作组完成回调
    void* user;                                         ///< 回调用户上下文
};

/**
 * @brief 动作组目录项
 */
typedef struct
{
    ActionId id;           ///< 动作组 ID
    ActionGroupDef group;  ///< 动作组定义
} ActionEntry;

/* ==========================================================================
 * Utility macros
 * ========================================================================== */

/** @brief 计算静态数组长度 */
#define ACTION_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/**
 * @brief 构造标签参数包
 * @param  tags_v:  标签数组指针
 * @param  count_v: 标签数量
 */
#define ACTION_TAG_ARGS(tags_v, count_v) tags_v, count_v

/**
 * @brief 不注册任何资源标签
 */
#define ACTION_NO_TAGS ACTION_TAG_ARGS(NULL, 0U)

/**
 * @brief 用数组构造标签参数包
 * @param  arr: const char *const 数组
 */
#define ACTION_TAGS(arr) ACTION_TAG_ARGS((arr), ACTION_ARRAY_SIZE(arr))

/* ==========================================================================
 * Value construction macros
 * ========================================================================== */

/**
 * @brief 构造无负载参数
 */
#define ACT_VAL_NONE() {.type = ACTION_VALUE_NONE}

/**
 * @brief 构造布尔值负载参数
 * @param  v: 布尔值
 */
#define ACT_VAL_BOOL(v) {.type = ACTION_VALUE_BOOL, .data.b = (v)}

/**
 * @brief 构造单浮点负载参数
 * @param  v: 浮点值
 */
#define ACT_VAL_FLOAT(v) {.type = ACTION_VALUE_FLOAT, .data.f = (v)}

/**
 * @brief 构造双浮点负载参数
 * @param  a_v: 第一个浮点值
 * @param  b_v: 第二个浮点值
 */
#define ACT_VAL_PAIR(a_v, b_v)                                      \
    {                                                               \
        .type = ACTION_VALUE_PAIR_FLOAT, .data.pair = {.a = (a_v),  \
                                                       .b = (b_v) } \
    }

/**
 * @brief 构造带名称的布尔值负载参数
 * @param  name_v: 名称字符串
 * @param  value_v: 布尔值
 */
#define ACT_VAL_NAMED_BOOL(name_v, value_v)                                       \
    {                                                                             \
        .type = ACTION_VALUE_NAMED_BOOL, .data.named_bool = {.name = (name_v),    \
                                                             .value = (value_v) } \
    }

/* ==========================================================================
 * Command construction macros
 * ========================================================================== */

/**
 * @brief 构造布尔命令
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  value_v:  布尔值
 */
#define ACT_CMD_BOOL(target_v, op_v, value_v) {.target = (target_v), .op = (op_v), .value = ACT_VAL_BOOL(value_v)}

/**
 * @brief 构造浮点命令
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  value_v:  浮点值
 */
#define ACT_CMD_FLOAT(target_v, op_v, value_v) {.target = (target_v), .op = (op_v), .value = ACT_VAL_FLOAT(value_v)}

/**
 * @brief 构造双浮点命令
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  a_v:      第一个浮点值
 * @param  b_v:      第二个浮点值
 */
#define ACT_CMD_PAIR(target_v, op_v, a_v, b_v) \
    {.target = (target_v), .op = (op_v), .value = ACT_VAL_PAIR(a_v, b_v)}

/**
 * @brief 构造带名称布尔命令
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  name_v:   名称字符串
 * @param  value_v:  布尔值
 */
#define ACT_CMD_NAMED_BOOL(target_v, op_v, name_v, value_v) \
    {.target = (target_v), .op = (op_v), .value = ACT_VAL_NAMED_BOOL(name_v, value_v)}

/* ==========================================================================
 * Wait construction macros
 * ========================================================================== */

/**
 * @brief 构造布尔等待条件
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  value_v:  期望布尔值
 */
#define ACT_WAIT_BOOL(target_v, op_v, value_v) \
    {.target = (target_v), .op = (op_v), .expected = ACT_VAL_BOOL(value_v)}

/**
 * @brief 构造浮点等待条件
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  value_v:  期望浮点值
 */
#define ACT_WAIT_FLOAT(target_v, op_v, value_v) \
    {.target = (target_v), .op = (op_v), .expected = ACT_VAL_FLOAT(value_v)}

/**
 * @brief 构造双浮点等待条件
 * @param  target_v: 当前实例内部端口
 * @param  op_v:     操作名字符串
 * @param  a_v:      第一个期望值
 * @param  b_v:      第二个期望值
 */
#define ACT_WAIT_PAIR(target_v, op_v, a_v, b_v) \
    {.target = (target_v), .op = (op_v), .expected = ACT_VAL_PAIR(a_v, b_v)}

/* ==========================================================================
 * Step and group construction macros
 * ========================================================================== */

/**
 * @brief 构造纯延时步骤
 * @param  dly_v: 延时 tick
 */
#define ACT_STEP_DELAY(dly_v) {.command_count = 0, .wait_count = 0, .dly = (dly_v)}

/** @brief 构造动作组定义 */
#define ACTION_GROUP_DEF_IMPL(step_arr, global_tags_v, global_count_v, local_tags_v, local_count_v, finish_cb, user_v) \
    {                                                                                                                  \
        .steps = (step_arr),                                                                                           \
        .step_count = ACTION_ARRAY_SIZE(step_arr),                                                                     \
        .global_tags = (global_tags_v),                                                                                \
        .global_tag_count = (global_count_v),                                                                          \
        .local_tags = (local_tags_v),                                                                                  \
        .local_tag_count = (local_count_v),                                                                            \
        .on_finish = (finish_cb),                                                                                      \
        .user = (user_v),                                                                                              \
    }

/** @brief 构造动作组定义 */
#define ACTION_GROUP_DEF(step_arr, global_tags_args, local_tags_args, finish_cb, user_v) \
    ACTION_GROUP_DEF_IMPL(step_arr, global_tags_args, local_tags_args, finish_cb, user_v)

/** @brief 构造无回调动作组注册项 */
#define ACTION_GROUP_ENTRY_IMPL(id_v, step_arr, global_tags_v, global_count_v, local_tags_v, local_count_v)        \
    {                                                                                                              \
        .id = (id_v),                                                                                              \
        .group = ACTION_GROUP_DEF_IMPL(step_arr, global_tags_v, global_count_v, local_tags_v, local_count_v, NULL, \
                                       NULL),                                                                      \
    }

/** @brief 构造无回调动作组注册项 */
#define ACTION_GROUP_ENTRY(id_v, step_arr, global_tags_args, local_tags_args) \
    ACTION_GROUP_ENTRY_IMPL(id_v, step_arr, global_tags_args, local_tags_args)

/** @brief 构造带回调动作组注册项 */
#define ACTION_GROUP_ENTRY_CB_IMPL(id_v, step_arr, global_tags_v, global_count_v, local_tags_v, local_count_v, \
                                   finish_cb, user_v)                                                          \
    {                                                                                                          \
        .id = (id_v),                                                                                          \
        .group = ACTION_GROUP_DEF_IMPL(step_arr, global_tags_v, global_count_v, local_tags_v, local_count_v,   \
                                       finish_cb, user_v),                                                     \
    }

/** @brief 构造带回调动作组注册项 */
#define ACTION_GROUP_ENTRY_CB(id_v, step_arr, global_tags_args, local_tags_args, finish_cb, user_v) \
    ACTION_GROUP_ENTRY_CB_IMPL(id_v, step_arr, global_tags_args, local_tags_args, finish_cb, user_v)

/** @brief 设备类型定义 */
struct ActionDevice {
    const ActionEntry* entries;                                                       ///< 该设备类型的动作组数组
    uint16_t entry_count;                                                             ///< 动作组数组长度
    ActionId null_id;                                                                 ///< 队列终止 ID
    bool (*is_available)(void* ctx);                                                  ///< 检查设备实例当前是否可执行，参数为实例 ctx，NULL 表示始终可用
    bool (*exec)(void* ctx, ActionQueue* queue, const ActionCommand* cmd);            ///< 执行命令，参数依次为实例 ctx、运行队列和命令
    bool (*is_done)(void* ctx, ActionQueue* queue, const ActionWaitCondition* wait);  ///< 检查等待条件，参数依次为实例 ctx、运行队列和等待条件
    void (*reset)(void* ctx);                                                         ///< 复位设备动作状态，参数为实例 ctx
};

/** @brief 动作执行实例注册项 */
typedef struct
{
    ActionExecId id;                       ///< 注册动作运行实例 ID
    ActionQueue queue;                     ///< 目标动作队列
    const ActionDeviceInstance* instance;  ///< 该执行目标绑定的设备实例
} ActionExecHandle;

/**
 * @brief 全局动作运行时配置
 */
typedef struct
{
    ActionExecHandle* exec_handles;  ///< 已注册动作运行实例列表
    uint8_t exec_handle_count;       ///< 已注册动作运行实例数量
} ActionRuntime;

extern ActionRuntime Action_Runtime;  ///< 全局动作运行时注册表

/**
 * @brief  启动指定执行实例的动作组队列
 * @param  id:   目标执行实例 ID
 * @param  ids:  动作组 ID 队列指针，以设备类的 null_id 结束
 * @retval true (启动成功)
 * @retval false (参数无效、队列忙、动作组缺失或资源冲突)
 */
bool Action_Start(ActionExecId id, const ActionId* ids);

/**
 * @brief  放行动作组间等待中的队列
 * @param  id: 目标执行实例 ID
 * @retval true (放行成功)
 * @retval false (实例不存在或当前不在 waiting 状态)
 */
bool Action_Skip(ActionExecId id);

/**
 * @brief  复位指定执行实例的动作队列和设备动作状态
 * @param  id: 目标执行实例 ID
 * @retval 无
 */
void Action_Reset(ActionExecId id);

/**
 * @brief  获取指定执行实例的队列状态
 * @param  id: 目标执行实例 ID
 * @retval ACTION_STATE_IDLE (队列空闲或实例不存在)
 * @retval ACTION_STATE_EXECUTING (队列执行中)
 */
ActionState Action_GroupState(ActionExecId id);

/**
 * @brief  获取指定执行实例的当前步骤状态
 * @param  id: 目标执行实例 ID
 * @retval ACTION_STATE_IDLE (当前步骤空闲或实例不存在)
 * @retval ACTION_STATE_EXECUTING (当前步骤执行中)
 */
ActionState Action_ActState(ActionExecId id);

/**
 * @brief  复位所有已注册动作队列和设备动作状态
 * @param  None
 * @retval 无
 */
void Action_ResetAll(void);

/**
 * @brief  更新所有已注册动作队列一次
 * @param  None
 * @retval 无
 */
void Action_Update(void);

/**
 * @brief  比较两个字符串是否相等
 * @param  a: 第一个字符串指针，允许为空
 * @param  b: 第二个字符串指针，允许为空
 * @retval true (两者均非空且内容相同)
 * @retval false (内容不同或至少一个为空)
 */
bool Action_StrEq(const char* a, const char* b);

#endif
