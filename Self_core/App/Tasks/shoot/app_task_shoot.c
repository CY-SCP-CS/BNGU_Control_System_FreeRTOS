/**
 * @file    app_task_shoot.c
 * @brief   发射命令快照及独立 5 ms 发射控制
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_shoot.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_can.h"
#include "app_task_telemetry.h"
#include "app_sentry_shoot.h"
#include <string.h>

static uint32_t s_sequence;
static osMutexId_t dbus_to_shoot_mutex;
static app_shoot_command_t s_shoot;

/**
 * @brief  读取一份完整的 已裁决发射命令快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由发射控制任务在任务上下文调用；使用 dbus_to_shoot_mutex，获取锁超时为 0。
 */
bool app_rtos_shoot_read(app_shoot_command_t *v)
{
    return app_rtos_snapshot_read(dbus_to_shoot_mutex, &s_shoot, v, sizeof(*v));
}

/**
 * @brief  发布 已裁决发射命令快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：ctrl_gim_Task；使用 dbus_to_shoot_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_shoot_write(const app_shoot_command_t *v)
{
    app_rtos_snapshot_write(dbus_to_shoot_mutex, &s_shoot, v, sizeof(*v));
}

/**
 * @brief  发射控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 5 ticks / 5 ms。
 * @note   读取云台裁决后的发射命令与电机反馈；摩擦轮、拨弹轮 PID 状态由本任务独占。
 */
void Start_ctrl_shoot_Task(void *argument)
{
    (void)argument;
#if APP_BOARD_HAS_SHOOT
    static app_shoot_input_t input;
    static app_shoot_output_t output;
    app_tasks_wait_start(TASK_SHOOT);
    uint32_t next = app_rtos_start_tick + APP_RTOS_SHOOT_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_SHOOT, &next, APP_RTOS_SHOOT_PERIOD);
        uint32_t started = app_rtos_cycles();
        /* 发射任务独占摩擦轮/拨弹轮 PID；不从 CAN 指令队列争抢消息。 */
        if (!app_rtos_shoot_read(&input.command))
        {
            app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        }
        if (!app_rtos_feedback_read(&input.feedback))
        {
            app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        }
        app_sentry_launcher_ctrl_200hz(&input, &output);
        uint32_t tick = osKernelGetTickCount();
        app_rtos_shoot_publish(&output, tick);
        app_ctrl_can_msg_t m = {.tick = tick,
                                .sequence = ++s_sequence,
                                .source = TASK_SHOOT,
                                .type = APP_CAN_SHOOT_CURRENT};
        memcpy(m.value.current, output.current, sizeof(output.current));
        app_rtos_ctrl_send(&m);
        app_tasks_profile_end(TASK_SHOOT, started);
        next += APP_RTOS_SHOOT_PERIOD;
    }
#else
    app_tasks_idle_failed(TASK_SHOOT);
#endif
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_shoot_init(void)
{
    s_sequence = 0;
    memset(&s_shoot, 0, sizeof(s_shoot));
    return app_rtos_object_mutex(&dbus_to_shoot_mutex, "dbus_to_shoot_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_shoot_deinit(void)
{
    app_rtos_object_mutex_delete(&dbus_to_shoot_mutex);
}
