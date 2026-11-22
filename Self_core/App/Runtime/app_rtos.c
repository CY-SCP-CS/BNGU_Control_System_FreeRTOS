/**
 * @file    app_rtos.c
 * @brief   通用启动状态、计数与运行诊断；任务装配在 Assembly
 */

#include "app_rtos.h"
#include "app_tasks.h"
#include "app_rtos_time.h"
#include "app_rtos_config.h"
#include "app_rtos_runtime_internal.h"
_Static_assert(APP_RTOS_KERNEL_TICK_HZ == 1000,
               "This application's periods require a 1 kHz kernel tick");
_Static_assert(APP_CONTROL_GIM_PERIOD_MS > 0 && APP_CONTROL_CHAS_PERIOD_MS > 0 &&
                   APP_CONTROL_SHOOT_PERIOD_MS > 0,
               "Control periods must be positive");
_Static_assert(APP_CONTROL_FEEDBACK_PERIOD_MS >= APP_CONTROL_GIM_PERIOD_MS &&
                   APP_CONTROL_FEEDBACK_PERIOD_MS % APP_CONTROL_GIM_PERIOD_MS == 0 &&
                   APP_CONTROL_FEEDBACK_PERIOD_MS >= APP_CONTROL_CHAS_PERIOD_MS &&
                   APP_CONTROL_FEEDBACK_PERIOD_MS % APP_CONTROL_CHAS_PERIOD_MS == 0,
               "Feedback period must be an integer multiple of control periods");
static osEventFlagsId_t sys_init_event;
#if APP_RTOS_ENABLE_EXAMPLES
static osSemaphoreId_t app_rtos_example_binary_sem; // 初始计数为 0，名称保持。
#endif
app_rtos_task_stats_t app_rtos_task_stats[TASK_COUNT];
volatile uint32_t app_rtos_counters[COUNT_COUNT];
uint32_t app_rtos_start_tick, app_rtos_init_stack_free;
uint8_t app_rtos_init_completed;

bool app_rtos_example_sem_release(void)
{
#if APP_RTOS_ENABLE_EXAMPLES
    return app_rtos_example_binary_sem && osSemaphoreRelease(app_rtos_example_binary_sem) == osOK;
#else
    return false;
#endif
}

bool app_rtos_example_sem_acquire(uint32_t timeout)
{
#if APP_RTOS_ENABLE_EXAMPLES
    return app_rtos_example_binary_sem &&
           osSemaphoreAcquire(app_rtos_example_binary_sem, timeout) == osOK;
#else
    (void)timeout;
    return false;
#endif
}

/** @brief 短临界区保护计数的读改写；允许任务和满足 RTOS 优先级限制的中断。 */
void app_rtos_count_add(unsigned index, uint32_t delta)
{
    uint32_t key = app_rtos_irq_lock();
    if (index < COUNT_COUNT)
    {
        app_rtos_counters[index] += delta;
    }
    app_rtos_irq_unlock(key);
}

void app_rtos_count(unsigned index)
{
    app_rtos_count_add(index, 1);
}

/** @brief 通过任务索引通知；未创建的任务忽略，标志 API 错误统一计数。 */
void app_rtos_notify_task(unsigned task, uint32_t flags)
{
    if (task >= TASK_COUNT)
    {
        return;
    }
    osThreadId_t handle = *app_rtos_task_table[task].handle;
    if (handle && (osThreadFlagsSet(handle, flags) & osFlagsError))
    {
        app_rtos_count(COUNT_FLAG_ERROR);
    }
}

void app_rtos_heartbeat(unsigned task)
{
    if (task >= TASK_COUNT)
    {
        return;
    }
    app_rtos_task_stats[task].heartbeat = osKernelGetTickCount();
    app_rtos_task_stats[task].heartbeat_count++;
}

/** @brief 持久状态查询，CMSIS 错误值的高位不能被误判为全部 READY 位。 */
uint32_t app_rtos_init_status(void)
{
    uint32_t flags = sys_init_event ? osEventFlagsGet(sys_init_event) : osFlagsErrorResource;
    return flags & osFlagsError ? 0 : flags;
}

/** @brief 初始化任务发布持久事件；所有接收者等待同一事件组。 */
void app_rtos_publish_init(uint32_t flags)
{
    /* 事件组保存启动状态，所有任务等待同一事实；不再逐个发送线程标志。 */
    const uint32_t allowed =
        APP_RTOS_EVT_PORT_READY | APP_RTOS_EVT_SYS_READY | APP_RTOS_EVT_SYS_INIT_FAILED;
    if ((flags & allowed) && (osEventFlagsSet(sys_init_event, flags & allowed) & osFlagsError))
    {
        app_rtos_count(COUNT_FLAG_ERROR);
    }
}

/** @brief 在所属一次性任务内调用；失效句柄不会再交给栈查询 API。 */
void app_rtos_task_complete(unsigned task)
{
    if (task >= TASK_COUNT)
    {
        return;
    }
    const app_rtos_task_desc_t *d = &app_rtos_task_table[task];
    if (!d->one_shot)
    {
        return;
    }
    uint32_t free_stack = *d->handle ? osThreadGetStackSpace(*d->handle) : 0;
    app_rtos_task_stats[task].final_stack_free = free_stack;
    app_rtos_task_stats[task].completed = 1;
    if (task == TASK_INIT)
    {
        app_rtos_init_stack_free = free_stack;
        app_rtos_init_completed = 1;
    }
    *d->handle = NULL;
}

/** @brief 等待持久的启动事件；osFlagsNoClear 保证晚启动任务也能看到状态。 */
void app_tasks_wait_phase(unsigned task, uint32_t wanted)
{
    if (!wanted)
    {
        return;
    }
    for (;;)
    {
        app_rtos_heartbeat(task);
        uint32_t flags = osEventFlagsWait(sys_init_event, wanted | APP_RTOS_EVT_SYS_INIT_FAILED,
                                          osFlagsWaitAny | osFlagsNoClear, APP_RTOS_EVENT_WAIT);
        if (flags & osFlagsError)
        {
            if (flags != osFlagsErrorTimeout)
            {
                app_rtos_count(COUNT_FLAG_ERROR);
                osDelay(1);
            }
            continue;
        }
        if ((flags & APP_RTOS_EVT_SYS_INIT_FAILED) &&
            !app_rtos_task_table[task].diagnostic_on_failure)
        {
            app_tasks_idle_failed(task);
        }
        return;
    }
}

/* Startup-only primitives; Assembly owns the higher-level rollback order. */
bool app_rtos_startup_objects_init(void)
{
    const osEventFlagsAttr_t event_attr = {.name = "sys_init_event"};
    sys_init_event = osEventFlagsNew(&event_attr);
    if (!sys_init_event)
    {
        return false;
    }
#if APP_RTOS_ENABLE_EXAMPLES
    const osSemaphoreAttr_t sem_attr = {.name = "app_rtos_example_binary_sem"};
    app_rtos_example_binary_sem = osSemaphoreNew(1, 0, &sem_attr);
    if (!app_rtos_example_binary_sem)
    {
        return false;
    }
#endif
    return true;
}
void app_rtos_startup_objects_deinit(void)
{
    if (sys_init_event)
    {
        osEventFlagsDelete(sys_init_event);
        sys_init_event = NULL;
    }
#if APP_RTOS_ENABLE_EXAMPLES
    if (app_rtos_example_binary_sem)
    {
        osSemaphoreDelete(app_rtos_example_binary_sem);
        app_rtos_example_binary_sem = NULL;
    }
#endif
}
