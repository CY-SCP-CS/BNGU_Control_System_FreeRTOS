/**
 * @file    app_task_dbus.c
 * @brief   DBUS 接收队列、解包与遥控快照
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_dbus_internal.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_control.h"
#include "app_task_monitor.h"
#include <string.h>
#include "lib_math.h"
#include "app_devices.h"

typedef struct
{
    uint8_t bytes[18];
    uint32_t tick, sequence;
} app_rtos_dbus_frame_t;

static osMessageQueueId_t dbus_rx_queue;
static osMutexId_t dbus_to_gim_mutex;
static app_dbus_snapshot_t s_dbus;
static uint32_t s_dbus_seq;

/** @brief 驱动临时 DMA 字节在回调返回前复制，不在中断解包或使用互斥量。 */
void app_rtos_dbus_receive_isr(const uint8_t bytes[18])
{
    if (!bytes)
    {
        app_rtos_count(COUNT_UART_ERROR);
        app_rtos_notify_task(TASK_DBUS, APP_RTOS_TF_DATA_READY);
        return;
    }
    app_rtos_dbus_frame_t frame = {.tick = osKernelGetTickCount(), .sequence = ++s_dbus_seq};
    memcpy(frame.bytes, bytes, sizeof(frame.bytes));
    if (osMessageQueuePut(dbus_rx_queue, &frame, 0, 0) != osOK)
    {
        app_rtos_count(COUNT_DBUS_DROP);
    }
    else
    {
        app_rtos_notify_task(TASK_DBUS, APP_RTOS_TF_DATA_READY);
    }
}

/**
 * @brief  读取一份完整的 DBUS 遥控快照
 * @param  v            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及诊断读取者在任务上下文调用；使用 dbus_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_dbus_read(app_dbus_snapshot_t *v)
{
    return app_rtos_snapshot_read(dbus_to_gim_mutex, &s_dbus, v, sizeof(*v));
}

/**
 * @brief  发布 DBUS 遥控快照
 * @param  v            输入快照，函数在锁内复制其内容
 * @note   写入者：comm_dbus_Task；使用 dbus_to_gim_mutex，仅允许任务上下文调用。
 * @note   等待取得锁后只复制；参数/句柄无效或获取失败时忽略本次更新。
 */
void app_rtos_dbus_write(const app_dbus_snapshot_t *v)
{
    app_rtos_snapshot_write(dbus_to_gim_mutex, &s_dbus, v, sizeof(*v));
}

/**
 * @brief  遥控接收与解包任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；PORT_READY 后处理完整 18 字节帧，事件等待最长 50 ticks。
 * @note   执行解包、合法性校验和归一化，发布遥控快照；短帧对齐与接收重启由驱动端口处理。
 */
void Start_comm_dbus_Task(void *argument)
{
    (void)argument;
#if APP_BOARD_HAS_DBUS
    static app_dbus_snapshot_t snapshot;
    app_tasks_wait_start(TASK_DBUS);
    for (;;)
    {
        app_rtos_dbus_frame_t frame;
        for (unsigned n = 0; n < APP_RTOS_DBUS_PARSE_BUDGET &&
                             osMessageQueueGet(dbus_rx_queue, &frame, NULL, 0) == osOK;
             n++)
        {
            uint32_t started = app_rtos_cycles();
            drv_dbus_data_t data;
            drv_dbus_decode(frame.bytes, &data);
            /* 非法帧不覆盖旧摇杆值，但发布 valid=0；有效帧才更新输入和有效数据时间。 */
            snapshot.valid = app_control_dbus_valid(&data);
            snapshot.sequence = frame.sequence;
            if (snapshot.valid)
            {

                snapshot.tick = frame.tick;
                snapshot.switch_s1 = data.rc.s1;
                snapshot.switch_s2 = data.rc.s2;
                for (unsigned i = 0; i < 4; ++i)
                {
                    snapshot.channel_norm[i] = lib_clamp(
                        ((float)data.rc.ch[i] - DRV_DBUS_CHANNEL_CENTER) / DRV_DBUS_CHANNEL_RANGE,
                        -1.0f, 1.0f);
                }
                (void)app_gimbal_comm_dbus_rx(&data, &snapshot.input);
            }
            else
            {
                app_rtos_count(COUNT_DBUS_INVALID);
            }
            app_rtos_dbus_write(&snapshot);
            app_rtos_module_update(MODULE_DBUS, snapshot.valid, snapshot.tick);
            app_tasks_profile_end(TASK_DBUS, started);
        }
        if (!app_devices_dbus_poll())
        {
            app_rtos_count(COUNT_IO_RESTART_ERROR);
        }
        app_tasks_wait_event(TASK_DBUS, APP_RTOS_EVENT_WAIT);
    }
#else
    app_tasks_idle_failed(TASK_DBUS);
#endif
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_dbus_init(void)
{
    s_dbus_seq = 0;
    memset(&s_dbus, 0, sizeof(s_dbus));
    return app_rtos_object_queue(&dbus_rx_queue, "dbus_rx_queue", APP_RTOS_DBUS_QUEUE_DEPTH,
                                 sizeof(app_rtos_dbus_frame_t)) &&
           app_rtos_object_mutex(&dbus_to_gim_mutex, "dbus_to_gim_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_dbus_deinit(void)
{
    app_rtos_object_queue_delete(&dbus_rx_queue);
    app_rtos_object_mutex_delete(&dbus_to_gim_mutex);
}
