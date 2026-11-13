/**
 * @file    app_task_log.c
 * @brief   UART1 日志独占者：定长事件、低频内存摘要或 VOFA 波形
 * @note    生产者不格式化、不等待；报告每轮最多一行，发送缓冲活到完成或确认中止。
 */
#include "app_rtos.h"
#include "app_task_log_internal.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_rtos_time.h"
#include "app_task_log_port.h"
#if APP_LOG_VOFA
#include "app_task_telemetry.h"
#include "drv_vofa.h"
#else
#include "app_task_health.h"
#include <stdio.h>
#endif

static osMessageQueueId_t sys_log_queue;

/**
 * @brief  按输出模式和等级筛选后，非阻塞投递值副本
 * @note   VOFA 不输出文本，直接过滤，不执行无意义的入队/出队。
 *          被配置过滤不计为丢包；实际满队列才累计 COUNT_LOG_DROP。
 */
void app_rtos_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b)
{
#if APP_LOG_VOFA
    (void)source;
    (void)level;
    (void)code;
    (void)a;
    (void)b;
#else
    if (level < APP_LOG_MIN_LEVEL)
    {
        return;
    }
    app_rtos_log_msg_t event = {osKernelGetTickCount(), code, source, level, a, b};
    if (osMessageQueuePut(sys_log_queue, &event, 0, 0) != osOK)
    {
        app_rtos_count(COUNT_LOG_DROP);
    }
#endif
}

/** @brief 无法确认缓冲已回收时停止复用；端口不隐藏无限循环。 */
static bool log_send(uint8_t *buffer, uint16_t length)
{
    app_log_tx_result_t result = app_rtos_uart_send(buffer, length);
    if (result == APP_LOG_TX_BUFFER_BUSY)
    {
        app_rtos_fatal_code = 4;
        app_tasks_idle_failed(TASK_LOG);
    }
    return result == APP_LOG_TX_OK;
}

#if !APP_LOG_VOFA
/**
 * @brief  发送一行已格式化文本；截断时也保留换行，避免多条输出粘连
 * @param  buffer   本任务独占的静态缓冲
 * @param  capacity 包含字符串终止符的缓冲大小，至少 3 字节
 * @param  length   snprintf() 返回值
 * @return true=发送完成，false=格式化或发送失败
 * @note   不发送字符串终止符；发送期间不能改写缓冲。
 */
static bool log_text_line(uint8_t *buffer, size_t capacity, int length)
{
    if (length <= 0 || capacity < 3)
    {
        return false;
    }
    size_t count = (size_t)length;
    if (count >= capacity)
    {
        count = capacity - 1;
        buffer[count - 2] = '\r';
        buffer[count - 1] = '\n';
    }
    app_rtos_heartbeat(TASK_LOG);
    return log_send(buffer, (uint16_t)count);
}

/**
 * @brief  每次仅发送一行内存报告，下一轮先服务普通事件再继续
 * @param  memory 紧凑的同一份内存采样，报告期间不修改
 * @param  cursor 0=堆摘要，1..TASK_COUNT=任务栈行
 * @param  detail 是否在摘要后继续输出启用任务的详细栈
 * @return true=仍有后续行，false=报告结束或发送失败
 * @note   最小栈只比较活动任务；详细表保留已退出初始化任务的最终水位。
 */
static bool log_memory_report(uint8_t *buffer, size_t capacity, const app_health_memory_t *memory,
                              unsigned *cursor, bool detail)
{
    int length;
    if (*cursor == 0)
    {
        uint32_t minimum = UINT32_MAX;
        const char *name = "none";
        for (unsigned i = 0; i < TASK_COUNT; i++)
        {
            const app_rtos_task_desc_t *d = &app_rtos_task_table[i];
            if (d->enabled && !memory->completed[i] && memory->stack_free[i] < minimum)
            {
                minimum = memory->stack_free[i];
                name = d->name;
            }
        }
        length = snprintf((char *)buffer, capacity,
                          "[MEM %lu] heap=%lu/%lu min=%lu stack_min=%lu task=%s drop=%lu B\r\n",
                          (unsigned long)memory->tick, (unsigned long)memory->heap_free,
                          (unsigned long)memory->heap_total, (unsigned long)memory->heap_min,
                          (unsigned long)(minimum == UINT32_MAX ? 0 : minimum), name,
                          (unsigned long)memory->log_drop);
        *cursor = 1;
        return log_text_line(buffer, capacity, length) && detail;
    }
    unsigned i = *cursor - 1;
    while (i < TASK_COUNT && !app_rtos_task_table[i].enabled)
    {
        i++;
    }
    if (i == TASK_COUNT)
    {
        return false;
    }
    const app_rtos_task_desc_t *d = &app_rtos_task_table[i];
    uint32_t free_min = memory->stack_free[i] < d->stack ? memory->stack_free[i] : d->stack;
    length = snprintf((char *)buffer, capacity,
                      "[STACK %lu] %s total=%lu used_max=%lu free_min=%lu %s\r\n",
                      (unsigned long)memory->tick, d->name, (unsigned long)d->stack,
                      (unsigned long)(d->stack - free_min), (unsigned long)free_min,
                      memory->completed[i] ? "EXITED" : "ACTIVE");
    *cursor = i + 2;
    return log_text_line(buffer, capacity, length) && *cursor <= TASK_COUNT;
}
#endif

/**
 * @brief  UART1 唯一发送任务
 * @param  argument CMSIS 任务参数，当前不使用
 * @note   TEXT：事件输出；初始化结束后一次详细栈表，之后默认每 5 秒一行摘要。
 *          VOFA：SYS_READY 后每 5 ms 九通道，字节流保持原格式。
 *          内存报告不占事件队列，详细表分轮发送，不能连续占用整个发送循环。
 */
void Start_sys_log_Task(void *argument)
{
    (void)argument;
    app_tasks_wait_start(TASK_LOG);
#if APP_LOG_VOFA
    static uint8_t buffer[DRV_VOFA_FRAME_SIZE];
    static float channels[9];
    app_tasks_wait_phase(TASK_LOG, APP_RTOS_EVT_SYS_READY);
    uint32_t next = app_rtos_start_tick + APP_RTOS_VOFA_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_LOG, &next, APP_RTOS_VOFA_PERIOD);
        uint32_t started = app_rtos_cycles();
        if (!app_rtos_vofa_read(channels))
        {
            app_rtos_task_stats[TASK_LOG].snapshot_failures++;
        }
        (void)drv_vofa_pack(channels, buffer);
        (void)log_send(buffer, DRV_VOFA_FRAME_SIZE);
        app_tasks_profile_end(TASK_LOG, started);
        next += APP_RTOS_VOFA_PERIOD;
    }
#else
    static uint8_t buffer[160];
    static app_health_memory_t memory;
    bool startup_pending = true;
    bool report_active = false;
    bool detail = false;
    unsigned cursor = 0;
    uint32_t memory_report = osKernelGetTickCount() + APP_RTOS_LOG_MEMORY_PERIOD;
    for (;;)
    {
        app_rtos_heartbeat(TASK_LOG);
        app_rtos_log_msg_t event;
        osStatus_t status = osMessageQueueGet(sys_log_queue, &event, NULL, APP_RTOS_EVENT_WAIT);
        uint32_t started = app_rtos_cycles();
        if (status == osOK)
        {
            int length = snprintf(
                (char *)buffer, sizeof(buffer), "[%lu] src=%u lvl=%u code=%u a=%ld b=%ld\r\n",
                (unsigned long)event.tick, (unsigned)event.source, (unsigned)event.level,
                (unsigned)event.code, (long)event.value0, (long)event.value1);
            (void)log_text_line(buffer, sizeof(buffer), length);
        }
        uint32_t now = osKernelGetTickCount();
        uint32_t state = app_rtos_init_status();
        bool initialized = (state & (APP_RTOS_EVT_SYS_READY | APP_RTOS_EVT_SYS_INIT_FAILED)) != 0;
        if (!report_active && initialized &&
            (startup_pending || (int32_t)(now - memory_report) >= 0))
        {
            if (app_rtos_health_memory_read(&memory))
            {
                detail = startup_pending || APP_LOG_MEMORY_DETAIL;
                startup_pending = false;
                cursor = 0;
                report_active = true;
                /* 从本次开始算下个期限，拥塞后只输出当前采样，不补发旧报告。 */
                memory_report = now + APP_RTOS_LOG_MEMORY_PERIOD;
            }
            else
            {
                app_rtos_task_stats[TASK_LOG].snapshot_failures++;
            }
        }
        if (report_active)
        {
            report_active = log_memory_report(buffer, sizeof(buffer), &memory, &cursor, detail);
        }
        app_tasks_profile_end(TASK_LOG, started);
    }
#endif
}

/** @brief 创建固定事件队列；VOFA 下也保留预留对象，不改容量和命名。 */
bool app_task_log_init(void)
{
    return app_rtos_object_queue(&sys_log_queue, "sys_log_queue", APP_RTOS_LOG_QUEUE_DEPTH,
                                 sizeof(app_rtos_log_msg_t));
}

/** @brief 仅调度器启动前失败回滚时删除对象。 */
void app_task_log_deinit(void)
{
    app_rtos_object_queue_delete(&sys_log_queue);
}
