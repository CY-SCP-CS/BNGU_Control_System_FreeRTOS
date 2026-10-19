/**
 * @file    app_rtos.c
 * @brief   显式任务配置表、对象创建、启动状态与应用诊断
 */

#include "app_task_init.h"
#include "app_task_chassis.h"
#include "app_task_gimbal.h"
#include "app_rtos_kernel.h"
#include "app_task_monitor.h"
#include "app_task_telemetry.h"
#include "app_task_log.h"
#include "app_task_health.h"
#include "app_task_referee.h"
#include "app_task_shoot.h"
#include "app_task_imu.h"
#include "app_task_dbus.h"
#include "app_task_can.h"
#include "app_rtos.h"
#include "app_tasks.h"
#include "app_rtos_time.h"
#include "app_board_config.h"
#include "app_rtos_config.h"

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
static osSemaphoreId_t app_rtos_example_binary_sem; //示例仍创建，保留原名，初始计数为 0。
/* 任务句柄：定义时为空，app_rtos_task_init() 创建线程后写入。 */
osThreadId_t ctrl_chas_TaskHandle;
osThreadId_t ctrl_gim_TaskHandle;
osThreadId_t ctrl_shoot_TaskHandle;
osThreadId_t comm_can_TaskHandle;
osThreadId_t comm_dbus_TaskHandle;
osThreadId_t comm_ref_TaskHandle;
osThreadId_t comm_imu_TaskHandle;
osThreadId_t sys_init_TaskHandle;
osThreadId_t sys_health_TaskHandle;
osThreadId_t sys_log_TaskHandle;

/* 统一任务配置：创建、启动门控和健康监测均读取此表。
 * [TASK_*] 指定数组下标，.成员名 指定结构体字段，无需依赖书写顺序。
 * 修改已有任务的属性只需编辑对应项；新增任务还需补齐编号、入口、
 * 句柄和接口声明；各模块的初始化入口直接列在本文件中。
 */
const app_rtos_task_desc_t app_rtos_task_table[TASK_COUNT] = {
    /* 底盘控制 */
    [TASK_CHAS] =
        {
            .name = "ctrl_chas_Task",
            .handle = &ctrl_chas_TaskHandle,
            .entry = Start_ctrl_chas_Task,
            .stack = 1536,
            .priority = osPriorityHigh,
            .enabled = APP_BOARD_HAS_CHASSIS,
            .start_flags = APP_RTOS_EVT_SYS_READY,
        },
    /* 云台控制 */
    [TASK_GIM] =
        {
            .name = "ctrl_gim_Task",
            .handle = &ctrl_gim_TaskHandle,
            .entry = Start_ctrl_gim_Task,
            .stack = 1536,
            .priority = osPriorityHigh,
            .enabled = APP_BOARD_HAS_GIMBAL,
            .start_flags = APP_RTOS_EVT_SYS_READY,
        },
    /* 发射控制 */
    [TASK_SHOOT] =
        {
            .name = "ctrl_shoot_Task",
            .handle = &ctrl_shoot_TaskHandle,
            .entry = Start_ctrl_shoot_Task,
            .stack = 1024,
            .priority = osPriorityAboveNormal3,
            .enabled = APP_BOARD_HAS_SHOOT,
            .start_flags = APP_RTOS_EVT_SYS_READY,
        },
    /* CAN 通信 */
    [TASK_CAN] =
        {
            .name = "comm_can_Task",
            .handle = &comm_can_TaskHandle,
            .entry = Start_comm_can_Task,
            .stack = 1024,
            .priority = osPriorityHigh1,
            .enabled = APP_BOARD_HAS_CAN,
            .start_flags = APP_RTOS_EVT_PORT_READY,
        },
    /* 遥控接收 */
    [TASK_DBUS] =
        {
            .name = "comm_dbus_Task",
            .handle = &comm_dbus_TaskHandle,
            .entry = Start_comm_dbus_Task,
            .stack = 1024,
            .priority = osPriorityAboveNormal2,
            .enabled = APP_BOARD_HAS_DBUS,
            .start_flags = APP_RTOS_EVT_PORT_READY,
        },
    /* 裁判通信 */
    [TASK_REF] =
        {
            .name = "comm_ref_Task",
            .handle = &comm_ref_TaskHandle,
            .entry = Start_comm_ref_Task,
            .stack = 1024,
            .priority = osPriorityAboveNormal1,
            .enabled = APP_BOARD_HAS_REFEREE,
            .start_flags = APP_RTOS_EVT_PORT_READY,
        },
    /* IMU 处理 */
    [TASK_IMU] =
        {
            .name = "comm_imu_Task",
            .handle = &comm_imu_TaskHandle,
            .entry = Start_comm_imu_Task,
            .stack = 1536,
            .priority = osPriorityHigh2,
            .enabled = APP_BOARD_HAS_IMU,
            .start_flags = APP_RTOS_EVT_SYS_READY,
        },
    /* 系统初始化 */
    [TASK_INIT] =
        {
            .name = "sys_init_Task",
            .handle = &sys_init_TaskHandle,
            .entry = Start_sys_init_Task,
            .stack = 1024,
            .priority = osPriorityHigh3,
            .enabled = (APP_BOARD_ALL_MASK & APP_BOARD_CURRENT_MASK) != 0,
            .start_flags = 0,
            .one_shot = true,
        },
    /* 健康监测 */
    [TASK_HEALTH] =
        {
            .name = "sys_health_Task",
            .handle = &sys_health_TaskHandle,
            .entry = Start_sys_health_Task,
            .stack = 768,
            .priority = osPriorityBelowNormal,
            .enabled = (APP_BOARD_ALL_MASK & APP_BOARD_CURRENT_MASK) != 0,
            .start_flags = APP_RTOS_EVT_PORT_READY,
            .diagnostic_on_failure = true,
        },
    /* 日志输出 */
    [TASK_LOG] =
        {
            .name = "sys_log_Task",
            .handle = &sys_log_TaskHandle,
            .entry = Start_sys_log_Task,
            .stack = 1024,
            .priority = osPriorityLow,
            .enabled = (APP_BOARD_ALL_MASK & APP_BOARD_CURRENT_MASK) != 0,
            .start_flags = APP_RTOS_EVT_PORT_READY,
            .diagnostic_on_failure = true,
        },
};

app_rtos_task_stats_t app_rtos_task_stats[TASK_COUNT];
volatile uint32_t app_rtos_counters[COUNT_COUNT];
uint32_t app_rtos_start_tick, app_rtos_init_stack_free;
uint8_t app_rtos_init_completed;

bool app_rtos_example_sem_release(void)
{
    return app_rtos_example_binary_sem && osSemaphoreRelease(app_rtos_example_binary_sem) == osOK;
}

bool app_rtos_example_sem_acquire(uint32_t timeout)
{
    return app_rtos_example_binary_sem &&
           osSemaphoreAcquire(app_rtos_example_binary_sem, timeout) == osOK;
}

/** @brief 仅启动失败时回滚；调用前已经终止所有已创建线程。 */
static void delete_objects(void)
{
    app_task_telemetry_deinit();
    app_task_log_deinit();
    app_task_health_deinit();
    app_task_referee_deinit();
    app_task_shoot_deinit();
    app_task_imu_deinit();
    app_task_dbus_deinit();
    app_task_can_deinit();
    if (sys_init_event)
    {
        osEventFlagsDelete(sys_init_event);
        sys_init_event = NULL;
    }
    if (app_rtos_example_binary_sem)
    {
        osSemaphoreDelete(app_rtos_example_binary_sem);
        app_rtos_example_binary_sem = NULL;
    }
}

/** @brief 调度器启动前创建通信对象和本板任务，成功后重复调用不重新创建。 */
bool app_rtos_task_init(void)
{
    static bool initialized;
    if (initialized)
    {
        return true;
    }
    /* 禁止把首次装配当作运行中的热重启，回滚只覆盖尚未运行的线程。 */
    if (osKernelGetState() != osKernelReady)
    {
        return false;
    }
    const osEventFlagsAttr_t event_attr = {.name = "sys_init_event"};
    const osSemaphoreAttr_t sem_attr = {.name = "app_rtos_example_binary_sem"};
    sys_init_event = osEventFlagsNew(&event_attr);
    app_rtos_example_binary_sem = osSemaphoreNew(1, 0, &sem_attr);
    /* 所有对象在调度器启动前创建，预留的对象也保留。模块入口直接可读。 */
    if (!(sys_init_event && app_rtos_example_binary_sem &&
          app_task_can_init() &&      // 六个 CAN 队列和反馈锁。
          app_task_dbus_init() &&     // 遥控帧队列和遥控快照锁。
          app_task_imu_init() &&      // 样本队列和姿态快照锁。
          app_task_shoot_init() &&    // 已裁决发射命令的快照锁。
          app_task_referee_init() &&  // 裁判 FIFO 状态和裁判快照锁。
          app_task_health_init() &&   // 健康状态快照锁。
          app_task_log_init() &&      // 日志事件队列。
          app_task_telemetry_init())) // 控制状态和波形共用的快照锁。
    {
        goto failed;
    }
    app_rtos_monitor_init();
    for (unsigned i = 0; i < TASK_COUNT; i++)
    {
        const app_rtos_task_desc_t *d = &app_rtos_task_table[i];
        if (!d->enabled)
        {
            continue;
        }
        const osThreadAttr_t attr = {
            .name = d->name, .stack_size = d->stack, .priority = d->priority};
        *d->handle = osThreadNew(d->entry, NULL, &attr);
        if (!*d->handle)
        {
            goto failed;
        }
    }
    initialized = true;
    return true;
failed:
    for (unsigned i = 0; i < TASK_COUNT; i++)
    {
        if (*app_rtos_task_table[i].handle)
        {
            osThreadTerminate(*app_rtos_task_table[i].handle);
            *app_rtos_task_table[i].handle = NULL;
        }
    }
    delete_objects();
    return false;
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
