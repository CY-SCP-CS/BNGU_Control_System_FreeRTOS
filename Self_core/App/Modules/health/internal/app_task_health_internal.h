/**
 * @file    app_task_health_internal.h
 * @brief   health 模块的任务入口与生命周期，仅实现和系统装配使用。
 */
#ifndef APP_TASK_HEALTH_INTERNAL_H
#define APP_TASK_HEALTH_INTERNAL_H
#include "app_task_health.h"
void app_rtos_monitor_init(void); /* Assembly only, before receiver startup. */
/* Bound before scheduling; sink must be nonblocking and copy all arguments. */
typedef void (*app_health_log_sink_t)(uint8_t, uint8_t, uint16_t, int32_t, int32_t);
void app_task_health_set_log_sink(app_health_log_sink_t sink);

/**
 * @brief  运行健康监测任务。
 * @param[in] argument CMSIS 任务参数，当前创建时为 NULL，不保留或使用。
 * @return 无返回值；任务持续运行。
 * @note   由任务清单创建，业务调用者不直接执行此入口。
 */
void Start_sys_health_Task(void *argument);
/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用。
 */
bool app_task_health_init(void);
/**
 * @brief  删除启动阶段已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚调用，必须先终止已创建任务。
 */
void app_task_health_deinit(void);
#endif
