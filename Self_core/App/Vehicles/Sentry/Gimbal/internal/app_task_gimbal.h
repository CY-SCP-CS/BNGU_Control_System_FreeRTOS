/**
 * @file    app_task_gimbal.h
 * @brief   云台控制任务接口；周期等待、控制源裁决和输出交付由实现文件负责
 */
#ifndef APP_TASK_GIMBAL_H
#define APP_TASK_GIMBAL_H

/**
 * @brief  云台控制任务接口；周期等待、控制源裁决和输出交付由实现文件负责
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_ctrl_gim_Task(void *argument);

#endif
