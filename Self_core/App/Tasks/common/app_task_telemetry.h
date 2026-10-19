/**
 * @file    app_task_telemetry.h
 * @brief   控制遥测服务；汇总状态私有，消费者不依赖控制任务入口
 */
#ifndef APP_TASK_TELEMETRY_H
#define APP_TASK_TELEMETRY_H
#include <stdbool.h>
#include "app_ctrl_data.h"

bool app_rtos_ctrl_read(app_ctrl_state_t *value); /**< 读取完整控制遥测，超时 0。 */
bool app_rtos_vofa_read(float value[8]);          /**< 只复制八通道，不复制整份控制结果。 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *value, float roll, uint32_t tick);
void app_rtos_shoot_publish(const app_shoot_output_t *value, uint32_t tick);
void app_rtos_chassis_publish(const app_chassis_output_t *value, uint32_t tick);
/* 发布函数只更新所属字段，共用 ctrl_state_mutex，锁内无格式化/发送/计算。 */

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_telemetry_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_telemetry_deinit(void);

#endif
