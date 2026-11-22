/**
 * @file    app_task_health.c
 * @brief   统一模块在线、任务心跳、周期统计和内存诊断
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_health_internal.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_vehicle_config.h"
#include "app_rtos_time.h"
#include "app_rtos_kernel.h"
#include <string.h>

static app_health_log_sink_t s_log_sink;

void app_task_health_set_log_sink(app_health_log_sink_t sink)
{
    s_log_sink = sink;
}

static void health_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b)
{
    if (s_log_sink)
    {
        s_log_sink(source, level, code, a, b);
    }
}

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
 * @param[in] v            输入快照，函数在锁内复制其内容
 * @note   写入者：sys_health_Task；使用 sys_health_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 * @return 无返回值；参数/句柄无效或取锁失败时保留旧快照。
 */
static void app_rtos_health_write(const app_sys_health_snapshot_t *v)
{
    app_rtos_snapshot_write(sys_health_mutex, &s_health_snapshot, v, sizeof(*v));
}

/** @brief 只在短锁内提取内存字段；日志不再保存完整健康快照。 */
bool app_rtos_health_memory_read(app_health_memory_t *value)
{
    if (!value || osMutexAcquire(sys_health_mutex, 0) != osOK)
    {
        return false;
    }
    if (s_health_snapshot.heap_total == 0)
    {
        osMutexRelease(sys_health_mutex);
        return false;
    }
    value->tick = s_health_snapshot.memory_tick;
    value->heap_total = s_health_snapshot.heap_total;
    value->heap_free = s_health_snapshot.heap_free;
    value->heap_min = s_health_snapshot.heap_min;
    value->log_drop = s_health_snapshot.counters[COUNT_LOG_DROP];
    for (unsigned i = 0; i < TASK_COUNT; i++)
    {
        value->stack_free[i] = s_health_snapshot.task[i].stack_free;
        value->completed[i] = s_health_snapshot.task[i].completed;
    }
    osMutexRelease(sys_health_mutex);
    return true;
}

static app_sys_health_snapshot_t s_health; // 工作副本仅由健康任务修改
static uint32_t s_reported_counters[COUNT_COUNT], s_reported_misses[TASK_COUNT];

/**
 * @brief  在线状态每 20 ms 更新，变化立即记录，持续离线只在提醒期限记录
 * @param  index       模块编号
 * @param  first_ready 本次是否刚进入系统就绪，用于报告启动时已经离线的模块
 * @param  remind      持续故障提醒期限是否到达
 * @note   未启用模块不告警；只生成诊断，不改变控制策略。
 */
static void module(unsigned index, bool first_ready, bool remind)
{
    const app_rtos_module_desc_t *d = &app_rtos_module_table[index];
    app_rtos_module_status_t status = {0};
    (void)app_rtos_module_read(index, &status);
    app_health_module_t *m = &s_health.module[index];
    uint8_t online =
        d->enabled && status.valid && (uint32_t)(s_health.tick - status.tick) <= d->threshold;
    if (s_health.ready && d->enabled &&
        (m->online != online || (!online && (first_ready || remind))))
    {
        health_log(TASK_HEALTH, online ? 1 : 2, 100 + index, online,
                   online ? 0 : (int32_t)(s_health.tick - status.tick));
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
    uint32_t now = osKernelGetTickCount();
    uint32_t next = now + APP_RTOS_HEALTH_PERIOD;
    uint32_t report = now + APP_RTOS_HEALTH_REPORT_PERIOD;
    uint32_t memory_sample = now; // 第一轮检查即建立有效内存样本
    uint32_t reminder = now + APP_RTOS_HEALTH_REMIND_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_HEALTH, &next, APP_RTOS_HEALTH_PERIOD);
        uint32_t started = app_rtos_cycles();
        s_health.tick = osKernelGetTickCount();
        uint32_t flags = app_rtos_init_status();
        bool first_ready = !s_health.ready && (flags & APP_RTOS_EVT_SYS_READY);
        s_health.ready = (flags & APP_RTOS_EVT_SYS_READY) != 0;
        s_health.init_failed = (flags & APP_RTOS_EVT_SYS_INIT_FAILED) != 0;
        bool sample_memory = (int32_t)(s_health.tick - memory_sample) >= 0;
        bool remind = (int32_t)(s_health.tick - reminder) >= 0;
        if (remind)
        {
            reminder +=
                ((uint32_t)(s_health.tick - reminder) / APP_RTOS_HEALTH_REMIND_PERIOD + 1U) *
                APP_RTOS_HEALTH_REMIND_PERIOD;
        }
        for (unsigned i = 0; i < APP_HEALTH_MODULE_COUNT; i++)
        {
            module(i, first_ready, remind);
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
            bool old_stack_warning = t->stack_warning;
            bool old_alive = t->alive;
            t->enabled = d->enabled;
            t->completed = d->one_shot && app_rtos_task_stats[i].completed;
            t->stats = app_rtos_task_stats[i];
            /* 栈水位需要扫描填充值，不能跟随 20 ms 心跳每次扫描。
             * 已退出初始化任务的缓存随时可读，不再查询失效句柄。 */
            if (t->completed)
            {
                t->stack_free = t->stats.final_stack_free;
            }
            else if (sample_memory)
            {
                t->stack_free = d->enabled && *d->handle ? osThreadGetStackSpace(*d->handle) : 0;
            }
            t->stack_warning =
                t->enabled && (t->stack_free < APP_RTOS_STACK_MIN_BYTES ||
                               t->stack_free < d->stack / APP_RTOS_STACK_RESERVE_DIVISOR);
            t->alive = t->enabled && (t->completed || !s_health.ready ||
                                      (uint32_t)(s_health.tick - t->stats.heartbeat) <=
                                          APP_RTOS_TASK_ALIVE_TIMEOUT);
            if (!t->enabled)
            {
                continue;
            }
            if (t->stack_warning != old_stack_warning || (remind && t->stack_warning))
            {
                health_log(TASK_HEALTH, t->stack_warning ? 2 : 1, 21, i, t->stack_free);
            }
            if (s_health.ready && (t->alive != old_alive || (remind && !t->alive)))
            {
                health_log(TASK_HEALTH, t->alive ? 1 : 2, 22, i,
                           t->alive ? 0 : (int32_t)(s_health.tick - t->stats.heartbeat));
            }
            if (t->stack_warning || !t->alive)
            {
                warnings++;
            }
        }
        bool old_heap_warning = s_health.heap_warning;
        if (sample_memory)
        {
            app_rtos_memory_t memory;
            app_rtos_memory_read(&memory);
            s_health.heap_total = memory.total;
            s_health.heap_free = memory.free;
            s_health.heap_min = memory.minimum_free;
            /* 配置堆包含管理开销；它不是整片 SRAM 的空闲量。 */
            s_health.heap_used = memory.free < memory.total ? memory.total - memory.free : 0;
            s_health.heap_warning = memory.free < APP_RTOS_HEAP_MIN_BYTES;
            s_health.memory_tick = s_health.tick;
            memory_sample +=
                ((uint32_t)(s_health.tick - memory_sample) / APP_RTOS_MEMORY_SAMPLE_PERIOD + 1U) *
                APP_RTOS_MEMORY_SAMPLE_PERIOD;
        }
        s_health.init_stack_free = app_rtos_init_stack_free;
        if (s_health.heap_warning != old_heap_warning || (remind && s_health.heap_warning))
        {
            health_log(TASK_HEALTH, s_health.heap_warning ? 2 : 1, 20, warnings,
                       s_health.heap_free);
        }
        app_rtos_health_write(&s_health);
        /* 仅对新增漏周期/错误汇总；正常状态不再每秒输出同一份报告。
         * 跳过过期汇总期限，不连续补发历史报告。 */
        if ((int32_t)(s_health.tick - report) >= 0)
        {
            report += ((uint32_t)(s_health.tick - report) / APP_RTOS_HEALTH_REPORT_PERIOD + 1U) *
                      APP_RTOS_HEALTH_REPORT_PERIOD;
            for (unsigned i = 0; i < TASK_COUNT; i++)
            {
                const app_health_task_t *t = &s_health.task[i];
                if (t->enabled && t->stats.missed != s_reported_misses[i])
                {
                    health_log(TASK_HEALTH, 2, 23, i, t->stats.missed - s_reported_misses[i]);
                }
                s_reported_misses[i] = t->stats.missed;
            }
            for (unsigned i = 0; i < COUNT_COUNT; i++)
            {
                if (s_health.counters[i] != s_reported_counters[i])
                {
                    /* 日志拥塞不再制造新的“日志丢失”日志；计数由内存摘要和调试器读取。 */
                    if (i != COUNT_LOG_DROP)
                    {
                        health_log(TASK_HEALTH, 2, 24, i,
                                   s_health.counters[i] - s_reported_counters[i]);
                    }
                    s_reported_counters[i] = s_health.counters[i];
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
    memset(s_reported_counters, 0, sizeof(s_reported_counters));
    memset(s_reported_misses, 0, sizeof(s_reported_misses));
    return app_rtos_object_mutex(&sys_health_mutex, "sys_health_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_health_deinit(void)
{
    app_rtos_object_mutex_delete(&sys_health_mutex);
}
