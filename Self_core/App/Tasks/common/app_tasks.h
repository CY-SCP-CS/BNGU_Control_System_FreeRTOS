/**
 * @file    app_tasks.h
 * @brief   App 共用的等待、周期和执行统计工具
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "app_rtos_types.h"

/* 任务入口在各任务自己的 app_task_<任务>.h 声明。
 * 此处从统一清单声明句柄；变量在 app_rtos.c 中由同一清单定义。
 */
#define APP_TASK(task_id, task_handle, task_entry, ...) \
    extern osThreadId_t task_handle;
#include "app_task_list.def"
#undef APP_TASK

/** @brief 等待任务表指定的首次启动阶段；失败后是否继续诊断也由表决定。 */
void app_tasks_wait_start(unsigned task);
/** @brief 额外等待另一阶段，例如 VOFA 在 PORT_READY 后再等待 SYS_READY。 */
void app_tasks_wait_phase(unsigned task, uint32_t wanted);
/** @brief 等待事件并维护心跳，timeout 单位 tick，标志不承担载荷存储。 */
void app_tasks_wait_event(unsigned task, uint32_t timeout);
/** @brief 按绝对 tick 等待；跳过过期节拍，记录漏周期，调用者返回后 next+=period。 */
void app_tasks_period_wait(unsigned task, uint32_t *next, uint32_t period);
/** @brief 记录一次业务轮数和最大经过时间，含抢占与阻塞。 */
void app_tasks_profile_end(unsigned task, uint32_t start);
/** @brief 初始化失败/误调用时保持心跳并等待，不返回。 */
void app_tasks_idle_failed(unsigned task);

#endif
