/**
 * @file    app_task_log_internal.h
 * @brief   log 模块的任务入口与生命周期，仅实现和系统装配使用。
 */
#ifndef APP_TASK_LOG_INTERNAL_H
#define APP_TASK_LOG_INTERNAL_H
#include "app_task_log.h"

/**
 * @brief  运行独占 UART1 的日志任务。
 * @param[in] argument CMSIS 任务参数，当前创建时为 NULL，不保留或使用。
 * @return 无返回值；任务持续运行。
 * @note   由任务清单创建，业务调用者不直接执行此入口。
 */
void Start_sys_log_Task(void *argument);
/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用。
 */
bool app_task_log_init(void);
/**
 * @brief  删除启动阶段已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚调用，必须先终止已创建任务。
 */
void app_task_log_deinit(void);
#endif
