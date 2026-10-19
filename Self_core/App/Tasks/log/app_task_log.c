/**
 * @file    app_task_log.c
 * @brief   UART1 日志独占者，TEXT 内存报告与 VOFA 发送
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_log.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_health.h"
#include "app_task_telemetry.h"
#include "app_task_log_port.h"
#include "drv_vofa.h"
#include <string.h>
#include <stdio.h>

static osMessageQueueId_t sys_log_queue;

/** @brief 投递值副本，满队列只计数，不阻塞控制/通信任务。 */
void app_rtos_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b)
{
    app_rtos_log_msg_t event = {osKernelGetTickCount(), code, source, level, a, b};
    if (osMessageQueuePut(sys_log_queue, &event, 0, 0) != osOK)
    {
        app_rtos_count(COUNT_LOG_DROP);
    }
}

/** @brief 日志任务决定无法回收发送缓冲时的策略，端口不隐藏无限循环。 */
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
 * @brief  发送一行已经格式化的文本，并维护日志任务心跳
 * @param  buffer       日志任务独占的静态发送缓冲
 * @param  capacity     缓冲容量，包含字符串终止符空间
 * @param  length       snprintf() 的返回值
 * @return true=本机发送完成，false=格式化或发送失败
 * @note   不发送字符串终止符；发送完成/确认中止后才允许改写缓冲。
 */
static bool log_text_line(uint8_t *buffer, size_t capacity, int length)
{
    if (length <= 0 || capacity < 2)
    {
        return false;
    }
    uint16_t count = (uint16_t)((size_t)length < capacity ? (size_t)length : capacity - 1);
    app_rtos_heartbeat(TASK_LOG);
    return log_send(buffer, count);
}

/**
 * @brief  输出一份堆及各业务任务栈的内存监控报告
 * @param  buffer       日志任务独占的静态发送缓冲
 * @param  capacity     发送缓冲容量
 * @return true=整份报告发送完成，false=快照读取或任意一行发送失败
 * @note   大快照使用静态 SRAM；锁内只复制，整数格式化和 UART 发送在锁外执行。
 * @note   单位为字节；free_min/used_max 是任务自创建以来的栈历史水位。
 * @note   heap_used 包含堆管理开销；任务栈已在 FreeRTOS 堆内分配，不能重复相加。
 */
static bool log_memory_report(uint8_t *buffer, size_t capacity)
{
    static app_sys_health_snapshot_t snapshot;
    if (!app_rtos_health_read(&snapshot))
    {
        app_rtos_task_stats[TASK_LOG].snapshot_failures++;
        return false;
    }
    int length = snprintf((char *)buffer, capacity,
                          "[MEM %lu] units=B heap_total=%lu used=%lu free=%lu min=%lu %s\r\n",
                          (unsigned long)snapshot.tick, (unsigned long)snapshot.heap_total,
                          (unsigned long)snapshot.heap_used, (unsigned long)snapshot.heap_free,
                          (unsigned long)snapshot.heap_min, snapshot.heap_warning ? "WARN" : "OK");
    if (!log_text_line(buffer, capacity, length))
    {
        return false;
    }
    for (unsigned i = 0; i < TASK_COUNT; i++)
    {
        const app_rtos_task_desc_t *d = &app_rtos_task_table[i];
        const app_health_task_t *t = &snapshot.task[i];
        if (!t->enabled)
        {
            length = snprintf((char *)buffer, capacity, "[STACK %lu] %s DISABLED\r\n",
                              (unsigned long)snapshot.tick, d->name);
        }
        else
        {
            /* 空闲量为历史最小值；初始化退出后仍显示健康任务保存的水位。 */
            uint32_t free_min = t->stack_free < d->stack ? t->stack_free : d->stack;
            uint32_t used_max = d->stack - free_min;
            uint32_t percent = d->stack ? free_min * 100U / d->stack : 0;
            const char *state = t->completed ? "EXITED" : t->alive ? "ALIVE" : "STALE";
            length =
                snprintf((char *)buffer, capacity,
                         "[STACK %lu] %s total=%lu used_max=%lu free_min=%lu (%lu%%) %s %s\r\n",
                         (unsigned long)snapshot.tick, d->name, (unsigned long)d->stack,
                         (unsigned long)used_max, (unsigned long)free_min, (unsigned long)percent,
                         state, t->stack_warning ? "WARN" : "OK");
        }
        /* 分行维护心跳；发送失败停止本轮，下一秒重新报告最新数据。 */
        if (!log_text_line(buffer, capacity, length))
        {
            return false;
        }
    }
    return true;
}
#endif
/**
 * @brief  UART1 日志与波形发送任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   两种板型均创建，独占 UART1；TEXT 输出日志事件并每秒输出堆/任务栈报告。
 * @note   VOFA 在 SYS_READY 后以 5 ms 周期发送原八通道波形。
 * @note   发送缓冲由本任务持有直到完成或确认中止；VOFA 字节流不混入文本。
 */
void Start_sys_log_Task(void *argument)
{
    (void)argument;
    static uint8_t buffer[160];
    app_tasks_wait_start(TASK_LOG);
#if APP_LOG_VOFA
    app_tasks_wait_phase(TASK_LOG, APP_RTOS_EVT_SYS_READY);
    static float channels[8];
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
        /* VOFA 串口保持纯二进制波形；消费日志事件释放队列，诊断仍保留于
         * 健康快照和计数器，不在波形流中混入 ASCII 文本。 */
        app_rtos_log_msg_t event;
        for (unsigned n = 0; n < APP_RTOS_LOG_QUEUE_DEPTH &&
                             osMessageQueueGet(sys_log_queue, &event, NULL, 0) == osOK;
             n++)
        {
        }
        app_tasks_profile_end(TASK_LOG, started);
        next += APP_RTOS_VOFA_PERIOD;
    }
#else
    const uint32_t report_period = osKernelGetTickFreq(); //一秒；当前内核频率 1000 Hz
    uint32_t memory_report = osKernelGetTickCount() + report_period;
    for (;;)
    {
        app_rtos_heartbeat(TASK_LOG);
        app_rtos_log_msg_t event;
        /* 无事件也必须检查报告期限；内存报告直接读取快照，不占用日志队列。 */
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
        if ((int32_t)(now - memory_report) >= 0)
        {
            (void)log_memory_report(buffer, sizeof(buffer));
            now = osKernelGetTickCount();
            /* 保持一秒相位；拥塞时跳过旧报告，避免连续补发历史数据。 */
            memory_report += ((uint32_t)(now - memory_report) / report_period + 1U) * report_period;
        }
        app_tasks_profile_end(TASK_LOG, started);
    }
#endif
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_log_init(void)
{

    return app_rtos_object_queue(&sys_log_queue, "sys_log_queue", APP_RTOS_LOG_QUEUE_DEPTH,
                                 sizeof(app_rtos_log_msg_t));
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_log_deinit(void)
{
    app_rtos_object_queue_delete(&sys_log_queue);
}
