/**
 * @file    app_tasks.h
 * @brief   App 共用的等待、周期和执行统计工具
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include "app_rtos_types.h"

/* 任务入口在各任务自己的 app_task_<任务>.h 声明。 */
/* 句柄在 app_rtos.c 中定义，由统一创建流程写入。 */
extern osThreadId_t ctrl_chas_TaskHandle;
extern osThreadId_t ctrl_gim_TaskHandle;
extern osThreadId_t ctrl_shoot_TaskHandle;
extern osThreadId_t comm_can_TaskHandle;
extern osThreadId_t comm_dbus_TaskHandle;
extern osThreadId_t comm_ref_TaskHandle;
extern osThreadId_t comm_imu_TaskHandle;
extern osThreadId_t sys_init_TaskHandle;
extern osThreadId_t sys_health_TaskHandle;
extern osThreadId_t sys_log_TaskHandle;

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
