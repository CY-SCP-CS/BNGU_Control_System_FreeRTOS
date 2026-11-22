/**
 * @file    app_task_shoot_internal.h
 * @brief   发射命令服务；保留 dbus_to_shoot_mutex 的名字和原语义
 */
#ifndef APP_TASK_SHOOT_INTERNAL_H
#define APP_TASK_SHOOT_INTERNAL_H
#include "app_task_shoot.h"

/**
 * @brief  发射控制任务入口；已裁决的发射输入快照接口见本头文件
 * @param[in] argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @return 无返回值；任务持续运行。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_ctrl_shoot_Task(void *argument);

/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=全部创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用，不支持运行中重置。
 */
bool app_task_shoot_init(void);
/**
 * @brief  删除本模块已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚使用，必须先终止所有已创建任务。
 */
void app_task_shoot_deinit(void);

#endif
