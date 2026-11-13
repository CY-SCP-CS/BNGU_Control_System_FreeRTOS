/**
 * @file    app_task_chassis.h
 * @brief   底盘控制任务接口；周期等待、输入组织和控制输出由实现文件负责
 */
#ifndef APP_TASK_CHASSIS_H
#define APP_TASK_CHASSIS_H

/**
 * @brief  底盘控制任务接口；周期等待、输入组织和控制输出由实现文件负责
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_ctrl_chas_Task(void *argument);

#endif
