/** @file app_task_dbus.h
 * @brief dbus 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_DBUS_H
#define APP_TASK_DBUS_H
#include <stdbool.h>
#include "app_dbus_data.h"
/**
 * @brief  读取一份完整的 DBUS 遥控快照
 * @param[out] value           输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由云台控制及诊断读取者在任务上下文调用；使用 dbus_to_gim_mutex，获取锁超时为 0。
 */
bool app_rtos_dbus_read(app_dbus_snapshot_t *value);

/* Service-owned resource capacities and per-cycle budgets. */
#define APP_RTOS_DBUS_QUEUE_DEPTH 4U
#define APP_RTOS_DBUS_PARSE_BUDGET 4U
#endif
