/** @file app_rtos_setup.c
 * @brief Static task and module composition; generic Runtime never includes modules.
 */
#include "app_task_init.h"
#include "app_rtos_setup.h"
#include "app_task_chassis.h"
#include "app_task_navigation.h"
#include "app_task_gimbal.h"
#include "app_rtos_kernel.h"
#include "app_task_monitor.h"
#include "app_task_telemetry_internal.h"
#include "app_task_log_internal.h"
#include "app_task_health_internal.h"
#include "app_task_referee_internal.h"
#include "app_task_shoot_internal.h"
#include "app_task_imu_internal.h"
#include "app_task_dbus_internal.h"
#include "app_task_can_internal.h"
#include "app_rtos.h"
#include "app_tasks.h"
#include "app_rtos_time.h"
#include "app_vehicle_config.h"
#include "app_rtos_config.h"

#include "app_task_log.h"
#include "app_rtos_runtime_internal.h"
/* 从统一清单定义句柄：静态初始化为 NULL，创建线程后写入线程 ID。 */
#define APP_TASK(task_id, task_handle, task_entry, ...) osThreadId_t task_handle;
#include "app_task_list.def"
#undef APP_TASK

/* 从同一清单构造任务表，创建、启动门控和健康监测共用。
 * 前三个参数提供编号、句柄和入口，__VA_ARGS__ 填入具名属性。
 * 任务配置在 App/Config/app_task_list.def 维护；入口声明由各任务头提供。
 */
#define APP_TASK(task_id, task_handle, task_entry, ...)                                            \
    [task_id] = {.handle = &task_handle, .entry = task_entry, __VA_ARGS__},

const app_rtos_task_desc_t app_rtos_task_table[TASK_COUNT] = {
#include "app_task_list.def"
};

#undef APP_TASK

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
    app_rtos_startup_objects_deinit();
}

/** @brief 调度器启动前创建通信对象和本板任务，成功后重复调用不重新创建。 */
bool app_rtos_create_tasks(void)
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
    if (!app_rtos_startup_objects_init())
    {
        goto failed;
    }
    /* 业务和预留通信对象仍全部创建；只有教学示例受独立开关控制。 */
    if (!(app_task_can_init() &&      // 三个 CAN 队列、输出邮箱锁和反馈锁。
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
    app_task_health_set_log_sink(app_rtos_log);
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
