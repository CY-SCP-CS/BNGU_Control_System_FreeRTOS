/**
 * @file    app_task_referee.c
 * @brief   裁判字节 FIFO、分批解析和完整快照
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_referee.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_monitor.h"
#include <string.h>

_Static_assert((APP_RTOS_REF_FIFO_SIZE & (APP_RTOS_REF_FIFO_SIZE - 1U)) == 0 &&
                   APP_RTOS_REF_FIFO_SIZE > 0,
               "Referee FIFO capacity must be a power of two");
static osMutexId_t ref_to_chas_mutex;
static app_ref_snapshot_t s_ref;

/**
 * @brief  读取一份完整的 裁判状态快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由需要裁判数据的任务（控制读取接口预留）在任务上下文调用；使用 ref_to_chas_mutex，获取锁超时为 0。
 */
bool app_rtos_ref_read(app_ref_snapshot_t *v)
{
    return app_rtos_snapshot_read(ref_to_chas_mutex, &s_ref, v, sizeof(*v));
}

/**
 * @brief  发布 裁判状态快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_ref_Task；使用 ref_to_chas_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_ref_write(const app_ref_snapshot_t *v)
{
    app_rtos_snapshot_write(ref_to_chas_mutex, &s_ref, v, sizeof(*v));
}

/**
 * @brief  裁判 ISR 到任务的私有字节 FIFO 状态。
 * @note   容量保持 1024 字节，通过 &1023 回绕；聚合状态不改变单生产者/单消费者规则。
 * @note   s_write 由 ISR 推进，s_read 由裁判任务推进；发布索引前保留内存屏障。
 */
typedef struct
{
    uint8_t s_fifo[APP_RTOS_REF_FIFO_SIZE]; /**< 有序字节存储，与 DMA 缓冲和解析半帧缓存独立。 */
    volatile uint32_t s_read;               /**< 消费累计计数，允许 uint32_t 自然回绕。 */
    volatile uint32_t s_write;              /**< 生产累计计数，s_write-s_read 为未消费字节数。 */
    volatile uint8_t s_reset;               /**< 溢出/错误复位请求，由任务在短临界区消费。 */
} app_ref_fifo_context_t;

static app_ref_fifo_context_t s_ref_fifo;

/**
 * @brief  把 USART6 DMA 接收片段追加到裁判 FIFO
 * @param  bytes        接收字节指针；NULL 表示 UART 错误并请求重同步
 * @param  n            本片段字节数；bytes 非 NULL 且 n=0 时忽略
 * @note   单生产者中断上下文调用，数据立即复制，不使用互斥量；中断优先级须允许 RTOS API。
 * @note   FIFO 满时丢弃本片段并置复位请求，等待消费者丢弃旧片段和重新同步；不越权改写消费者索引。
 */
void app_rtos_ref_push_isr(const uint8_t *bytes, uint16_t n)
{
    if (!bytes)
    {
        s_ref_fifo.s_reset = 1;
        app_rtos_count(COUNT_UART_ERROR);
        app_rtos_notify_task(TASK_REF, APP_RTOS_TF_DATA_READY);
        return;
    }
    if (!n)
    {
        return;
    }
    if (s_ref_fifo.s_reset)
    {
        return;
    }
    if (n > sizeof(s_ref_fifo.s_fifo) - (uint32_t)(s_ref_fifo.s_write - s_ref_fifo.s_read))
    {
        s_ref_fifo.s_reset = 1;
        app_rtos_count(COUNT_REF_OVERFLOW);
        /* 生产者只提出复位请求，不改写消费者索引；复位期间新片段暂不写入。 */
        app_rtos_notify_task(TASK_REF, APP_RTOS_TF_DATA_READY);
        return;
    }
    for (unsigned i = 0; i < n; i++)
    {
        s_ref_fifo.s_fifo[(s_ref_fifo.s_write + i) & (APP_RTOS_REF_FIFO_SIZE - 1U)] = bytes[i];
    }
    /* 先写完字节再发布 s_write，消费者看到新索引时才可读取对应内容。 */
    app_rtos_memory_barrier();
    s_ref_fifo.s_write += n;
    app_rtos_notify_task(TASK_REF, APP_RTOS_TF_DATA_READY);
}

/**
 * @brief  从裁判 FIFO 取出有序字节并消费复位请求
 * @param  bytes        输出缓冲，cap 大于 0 时必须有效
 * @param  cap          缓冲最大容量，单位字节
 * @param  reset        有效输出指针；1=解析器需先丢弃旧解析片段并重新同步
 * @return 本次复制的字节数，不超过 cap
 * @note   仅 comm_ref_Task 作为单消费者调用；短临界区处理复位，字节复制不持有互斥量。
 * @note   复位后可能已收到新字节，调用者应先处理 reset 再解析返回的字节。
 */
uint16_t app_rtos_ref_pop(uint8_t *bytes, uint16_t cap, uint8_t *reset)
{
    uint32_t key = app_rtos_irq_lock();
    *reset = s_ref_fifo.s_reset;
    if (s_ref_fifo.s_reset)
    {
        s_ref_fifo.s_read = s_ref_fifo.s_write;
        s_ref_fifo.s_reset = 0;
    }
    app_rtos_irq_unlock(key);

    uint32_t n = s_ref_fifo.s_write - s_ref_fifo.s_read;
    if (n > cap)
    {
        n = cap;
    }
    for (unsigned i = 0; i < n; i++)
    {
        bytes[i] = s_ref_fifo.s_fifo[(s_ref_fifo.s_read + i) & (APP_RTOS_REF_FIFO_SIZE - 1U)];
    }
    /* 先完成字节读取再发布 s_read，生产者才能复用这些 FIFO 空间。 */
    app_rtos_memory_barrier();
    s_ref_fifo.s_read += n;
    return (uint16_t)n;
}

/**
 * @brief  裁判系统组帧与解包任务入口。
 * @param  argument  CMSIS 任务参数，当前创建时传 NULL。
 * @note   仅底盘板创建；PORT_READY 后消费有序 FIFO，每批最多处理 8 段、每段 128 字节。
 * @note   解析函数返回本次更新掩码和计数，逐段维护字段时间，批末最多发布一份完整快照。
 * @note   接收恢复、心跳及最长 50 ticks 的事件等待保持原职责，不增加裁判限功率或限热量策略。
 */
void Start_comm_ref_Task(void *argument)
{
    (void)argument;
#if APP_BOARD_HAS_REFEREE
    static app_ref_snapshot_t snapshot;
    static uint8_t chunk[APP_RTOS_REF_CHUNK_SIZE];
    app_tasks_wait_start(TASK_REF);
    for (;;)
    {
        drv_referee_parse_result_t batch = {0};
        uint32_t last_frame_tick = 0;

        /* 字节处理保持有界；batch 汇总本轮结果，整份业务数据只在批末复制。 */
        for (unsigned pass = 0; pass < APP_RTOS_REF_BATCH_CHUNKS; pass++)
        {
            uint32_t started = app_rtos_cycles();
            uint8_t reset;
            uint16_t n = app_rtos_ref_pop(chunk, sizeof(chunk), &reset);
            if (reset)
            {
                drv_referee_reset_stream();
            }
            if (!n)
            {
                break;
            }

            drv_referee_parse_result_t result = drv_referee_process(chunk, n);
            app_tasks_profile_end(TASK_REF, started);
            batch.updated_mask |= result.updated_mask;
            batch.frames += result.frames;
            batch.errors += result.errors;

            if (result.frames)
            {
                last_frame_tick = osKernelGetTickCount();
                /* 保留逐段更新时间；最后一段若只更新其他字段，不会覆盖先前字段时间。 */
                for (unsigned i = 0; i < 20; i++)
                {
                    if (result.updated_mask & (1UL << i))
                    {
                        snapshot.field_tick[i] = last_frame_tick;
                    }
                }
            }
        }

        if (batch.frames)
        {
            /* 已有有效字段继续保留；本次发布的 updated_mask 是整批更新位的按位或。 */
            uint32_t valid_mask = snapshot.data.data_valid_flags | batch.updated_mask;
            snapshot.data = *drv_referee_get_data();
            snapshot.data.data_valid_flags = valid_mask;
            snapshot.updated_mask = batch.updated_mask;
            snapshot.tick = last_frame_tick;
            snapshot.sequence += batch.frames;
            snapshot.valid = 1;
            app_rtos_ref_write(&snapshot);
            app_rtos_module_update(MODULE_REFEREE, snapshot.valid, snapshot.tick);
        }
        if (batch.errors)
        {
            app_rtos_count_add(COUNT_REF_CRC, batch.errors);
        }
        /* poll 已在重启尝试前清理解析半帧，失败时只补充诊断计数。 */
        if (!drv_referee_port_poll())
        {
            app_rtos_count(COUNT_IO_RESTART_ERROR);
        }
        app_tasks_wait_event(TASK_REF, APP_RTOS_EVENT_WAIT);
    }
#else
    app_tasks_idle_failed(TASK_REF);
#endif
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_referee_init(void)
{
    memset(&s_ref, 0, sizeof(s_ref));
    memset(&s_ref_fifo, 0, sizeof(s_ref_fifo));
    return app_rtos_object_mutex(&ref_to_chas_mutex, "ref_to_chas_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_referee_deinit(void)
{
    app_rtos_object_mutex_delete(&ref_to_chas_mutex);
}
