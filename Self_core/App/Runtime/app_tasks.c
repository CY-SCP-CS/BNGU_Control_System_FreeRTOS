/**
 * @file    app_tasks.c
 * @brief   共用事件等待、绝对周期调度与执行统计；启动事件由 app_rtos.c 管理
 */

#include "app_tasks.h"
#include "app_rtos.h"
#include "app_rtos_time.h"
#include "app_rtos_config.h"
#include "lib_periodic.h"

/**
 * @brief  让初始化失败或误调用的业务任务停留在安全等待
 * @param  task         当前 TASK_* 索引
 * @note   持续维护心跳并等待 50 ticks，不执行业务；此函数不返回。
 */
void app_tasks_idle_failed(unsigned task)
{
    for (;;)
    {
        app_rtos_heartbeat(task);
        osDelay(APP_RTOS_EVENT_WAIT);
    }
}

/**
 * @brief  等待统一唤醒位并维护心跳，具体原因由队列和驱动状态保存
 * @param  task         当前 TASK_* 索引
 * @param  timeout      等待上限，单位内核 tick
 * @note   线程标志只负责唤醒；数据保存在队列/FIFO，多个事件合并不会丢失已入队的数据。
 */
void app_tasks_wait_event(unsigned task, uint32_t timeout)
{
    app_rtos_heartbeat(task);
    uint32_t f = osThreadFlagsWait(APP_RTOS_TF_DATA_READY, osFlagsWaitAny, timeout);
    if ((f & osFlagsError) && f != osFlagsErrorTimeout)
    {
        app_rtos_count(COUNT_FLAG_ERROR);
    }
}

/* 截止时间始终沿共同相位推进。当前 CMSIS 包装对已到/过去的 tick 返回
 * osErrorParameter：恰好到期直接执行，过期推进并计漏周期，真实 API 错误另行计数。 */
/**
 * @brief  等待下一绝对截止 tick，跳过已过期周期
 * @param  task         当前周期任务 TASK_* 索引
 * @param  next         输入/输出截止 tick；过期时推进到下一未来节拍
 * @param  period       周期 tick 数，必须大于 0
 * @note   目标恰好已到就执行；已经过期只计漏周期，不连续补算旧控制。
 * @note   区分 osErrorParameter 的到期竞争和真实 API 错误；所有 tick 间隔应小于 2^31。
 * @note   返回后由调用者 next += period，保持共同相位，不以业务完成时间重定周期。
 */
void app_tasks_period_wait(unsigned task, uint32_t *next, uint32_t period)
{
    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        app_rtos_task_stats[task].missed += lib_periodic_skip_expired(now, period, next);
        now = osKernelGetTickCount();
        if (now == *next)
        {
            break;
        }
        osStatus_t status = osDelayUntil(*next);
        if (status == osOK)
        {
            break;
        }
        now = osKernelGetTickCount();
        if (status == osErrorParameter && (int32_t)(now - *next) >= 0)
        {
            if (now == *next)
            {
                break;
            }
            continue;
        }
        app_rtos_task_stats[task].delay_errors++;
        osDelay(1);
    }
    uint32_t late = app_rtos_wakeup_delay_us(*next);
    if (late > app_rtos_task_stats[task].max_wakeup_us)
    {
        app_rtos_task_stats[task].max_wakeup_us = late;
    }
    app_rtos_heartbeat(task);
}

/**
 * @brief  统计本轮任务执行次数和最大耗时
 * @param  task         当前 TASK_* 索引
 * @param  start        本轮开始时读取的 DWT 周期计数
 * @note   经过时间包含执行期间的抢占和阻塞，不是独占 CPU 的算法耗时。
 */
void app_tasks_profile_end(unsigned task, uint32_t start)
{
    uint32_t elapsed = app_rtos_elapsed_us(start);
    if (elapsed > app_rtos_task_stats[task].max_execution_us)
    {
        app_rtos_task_stats[task].max_execution_us = elapsed;
    }
    app_rtos_task_stats[task].cycles++;
}

/** @brief 从单一任务表读取首次启动条件，避免每个入口重复硬编码。 */
void app_tasks_wait_start(unsigned task)
{
    app_tasks_wait_phase(task, app_rtos_task_table[task].start_flags);
}
