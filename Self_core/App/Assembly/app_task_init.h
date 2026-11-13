/**
 * @file    app_task_init.h
 * @brief   一次性系统初始化任务接口；编排业务、端口、标定和就绪发布
 */
#ifndef APP_TASK_INIT_H
#define APP_TASK_INIT_H

/**
 * @brief  一次性系统初始化任务接口；编排业务、端口、标定和就绪发布
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_sys_init_Task(void *argument);

#endif
