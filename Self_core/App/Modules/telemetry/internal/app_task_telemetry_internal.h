/**
 * @file    app_task_telemetry_internal.h
 * @brief   遥测生命周期；公共服务声明只在 app_task_telemetry.h 维护。
 */
#ifndef APP_TASK_TELEMETRY_INTERNAL_H
#define APP_TASK_TELEMETRY_INTERNAL_H
#include "app_task_telemetry.h"

/**
 * @brief  复位遥测快照并创建其互斥量。
 * @return true=创建成功；false=创建失败，由装配统一回滚。
 * @note   仅调度器启动前调用；快照由各控制任务分字段发布。
 */
bool app_task_telemetry_init(void);
/**
 * @brief  删除遥测快照互斥量。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚调用，必须先终止所有已创建任务。
 */
void app_task_telemetry_deinit(void);
#endif
