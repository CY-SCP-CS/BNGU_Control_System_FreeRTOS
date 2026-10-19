/**
 * @file    app_task_shoot.h
 * @brief   发射命令服务；保留 dbus_to_shoot_mutex 的名字和原语义
 */
#ifndef APP_TASK_SHOOT_H
#define APP_TASK_SHOOT_H
#include <stdbool.h>
#include "app_shoot_data.h"

bool app_rtos_shoot_read(app_shoot_command_t *value); /**< 发射任务读取裁决命令，失败保留旧值。 */
void app_rtos_shoot_write(
    const app_shoot_command_t *value); /**< 云台任务发布裁决结果，来源可为 DBUS/CAN。 */

/**
 * @brief  发射控制任务入口；已裁决的发射输入快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_ctrl_shoot_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_shoot_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_shoot_deinit(void);

#endif
