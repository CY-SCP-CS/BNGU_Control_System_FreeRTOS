/**
 * @file    app_task_health.c
 * @brief   统一模块在线、任务心跳、周期统计和内存诊断
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_health.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_log.h"
#include "app_rtos_kernel.h"
#include <string.h>

static osMutexId_t sys_health_mutex;
static app_sys_health_snapshot_t s_health_snapshot;

/**
 * @brief  读取一份完整的 系统健康快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由诊断查询任务在任务上下文调用；使用 sys_health_mutex，获取锁超时为 0。
 */
bool app_rtos_health_read(app_sys_health_snapshot_t *v)
{
    return app_rtos_snapshot_read(sys_health_mutex, &s_health_snapshot, v, sizeof(*v));
}

/**
 * @brief  发布 系统健康快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：sys_health_Task；使用 sys_health_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_health_write(const app_sys_health_snapshot_t *v)
{
    app_rtos_snapshot_write(sys_health_mutex, &s_health_snapshot, v, sizeof(*v));
}

static app_sys_health_snapshot_t s_health; //健康任务组装后整体发布，避免暴露半更新诊断表
static uint32_t s_reported_counters[COUNT_COUNT],
    s_reported_misses[TASK_COUNT]; //已报告计数水位，用于只输出新的变化
static uint8_t s_reported_stack_warning[TASK_COUNT],
    s_reported_alive[TASK_COUNT]; //上一轮已报告的告警/存活状态

/**
 * @brief  更新一个模块的在线状态并记录状态变化
 * @param  index        健康模块索引，调用者保证小于 APP_HEALTH_MODULE_COUNT
 * @note   使用健康任务本轮时间判断在线；未启用模块不告警，READY 后状态变化才产生日志。
 */
static void module(unsigned index)
{
    const app_rtos_module_desc_t *d = &app_rtos_module_table[index];
    app_rtos_module_status_t status = {0};
    (void)app_rtos_module_read(index, &status);
    app_health_module_t *m = &s_health.module[index];
    uint8_t online =
        d->enabled && status.valid && (uint32_t)(s_health.tick - status.tick) <= d->threshold;
    if (s_health.ready && m->enabled && m->online != online)
    {
        app_rtos_log(TASK_HEALTH, online ? 1 : 2, 100 + index, online, 0);
    }
    *m = (app_health_module_t){.name = d->name,
                               .tick = status.tick,
                               .threshold = d->threshold,
                               .enabled = d->enabled,
                               .valid = status.valid,
                               .online = online,
                               .errors = status.errors,
                               .overruns = status.overruns};
    if (index == MODULE_IMU)
    {
        s_health.imu_busy = status.overruns;
        s_health.imu_errors = status.errors;
    }
}

/**
 * @brief  系统健康监测任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建；PORT_READY 后使用 osDelayUntil()，周期 20 ticks / 20 ms。
 * @note   监测模块在线、心跳、漏周期、栈/堆和丢包错误；只告警，不改变原控制策略。
 */
void Start_sys_health_Task(void *argument)
{
    (void)argument;
    app_tasks_wait_start(TASK_HEALTH);
    uint32_t next = osKernelGetTickCount() + APP_RTOS_HEALTH_PERIOD;
    uint32_t report = osKernelGetTickCount() + APP_RTOS_HEALTH_REPORT_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_HEALTH, &next, APP_RTOS_HEALTH_PERIOD);
        uint32_t started = app_rtos_cycles();
        s_health.tick = osKernelGetTickCount();
        uint32_t flags = app_rtos_init_status();
        s_health.ready = (flags & APP_RTOS_EVT_SYS_READY) != 0;
        s_health.init_failed = (flags & APP_RTOS_EVT_SYS_INIT_FAILED) != 0;
        /* 设备生产者发布通用状态；本任务不再取得或解释各设备的大快照。 */
        for (unsigned i = 0; i < APP_HEALTH_MODULE_COUNT; i++)
        {
            module(i);
        }
        uint32_t key = app_rtos_irq_lock();
        for (unsigned i = 0; i < COUNT_COUNT; i++)
        {
            s_health.counters[i] = app_rtos_counters[i];
        }
        app_rtos_irq_unlock(key);
        unsigned warnings = 0;
        for (unsigned i = 0; i < TASK_COUNT; i++)
        {
            const app_rtos_task_desc_t *d = &app_rtos_task_table[i];
            app_health_task_t *t = &s_health.task[i];
            t->enabled = d->enabled;
            t->completed = d->one_shot && app_rtos_task_stats[i].completed;
            t->stats = app_rtos_task_stats[i];
            /* 当前 CMSIS 适配层已返回字节，不能再乘 4；初始化已退出则读取缓存水位。 */
            t->stack_free = t->completed ? t->stats.final_stack_free
                            : *d->handle ? osThreadGetStackSpace(*d->handle)
                                         : 0;
            /* 栈余量不足分配值 20% 或不足 128 字节即告警，未启用任务不告警。 */
            t->stack_warning =
                t->enabled && (t->stack_free < APP_RTOS_STACK_MIN_BYTES ||
                               t->stack_free < d->stack / APP_RTOS_STACK_RESERVE_DIVISOR);
            t->alive = t->enabled && (t->completed || !s_health.ready ||
                                      (uint32_t)(s_health.tick - t->stats.heartbeat) <=
                                          APP_RTOS_TASK_ALIVE_TIMEOUT);
            if (t->enabled && (t->stack_warning || !t->alive))
            {
                warnings++;
            }
        }
        app_rtos_memory_t memory;
        app_rtos_memory_read(&memory);
        s_health.heap_total = memory.total;
        s_health.heap_free = memory.free;
        s_health.heap_min = memory.minimum_free;
        /* 配置堆大小含管理/对齐开销；空闲堆不是整颗芯片的剩余 SRAM。 */
        s_health.heap_used =
            s_health.heap_free < s_health.heap_total ? s_health.heap_total - s_health.heap_free : 0;
        s_health.heap_warning = s_health.heap_free < APP_RTOS_HEAP_MIN_BYTES;
        s_health.init_stack_free = app_rtos_init_stack_free;
        app_rtos_health_write(&s_health);
        /* 20 ms 更新快照，1 秒汇总日志；通过已报告水位区分新错误与历史累计。 */
        if ((int32_t)(s_health.tick - report) >= 0)
        {
            report += APP_RTOS_HEALTH_REPORT_PERIOD;
            app_rtos_log(TASK_HEALTH, warnings || s_health.heap_warning ? 2 : 1, 20, warnings,
                         s_health.heap_free);
            for (unsigned i = 0; i < TASK_COUNT; i++)
            {
                if (s_health.task[i].enabled)
                {
                    app_health_task_t *t = &s_health.task[i];
                    if (t->stack_warning && !s_reported_stack_warning[i])
                    {
                        app_rtos_log(TASK_HEALTH, 2, 21, i, t->stack_free);
                    }
                    if (s_health.ready && !t->alive && s_reported_alive[i])
                    {
                        app_rtos_log(TASK_HEALTH, 2, 22, i, s_health.tick - t->stats.heartbeat);
                    }
                    if (t->stats.missed != s_reported_misses[i])
                    {
                        app_rtos_log(TASK_HEALTH, 2, 23, i, t->stats.missed - s_reported_misses[i]);
                    }
                    s_reported_stack_warning[i] = t->stack_warning;
                    s_reported_alive[i] = t->alive;
                    s_reported_misses[i] = t->stats.missed;
                }
            }
            for (unsigned i = 0; i < COUNT_COUNT; i++)
            {
                if (s_health.counters[i] != s_reported_counters[i])
                {
                    app_rtos_log(TASK_HEALTH, 2, 24, i,
                                 s_health.counters[i] - s_reported_counters[i]);
                    s_reported_counters[i] = s_health.counters[i];
                }
            }
            if (s_health.ready)
            {
                for (unsigned i = 0; i < APP_HEALTH_MODULE_COUNT; i++)
                {
                    if (s_health.module[i].enabled && !s_health.module[i].online)
                    {
                        app_rtos_log(TASK_HEALTH, 2, 100 + i, 0,
                                     s_health.tick - s_health.module[i].tick);
                    }
                }
            }
        }
        app_tasks_profile_end(TASK_HEALTH, started);
        next += APP_RTOS_HEALTH_PERIOD;
    }
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_health_init(void)
{
    memset(&s_health_snapshot, 0, sizeof(s_health_snapshot));
    memset(&s_health, 0, sizeof(s_health));
    return app_rtos_object_mutex(&sys_health_mutex, "sys_health_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_health_deinit(void)
{
    app_rtos_object_mutex_delete(&sys_health_mutex);
}
