/**
 * @file    app_task_can_port.h
 * @brief   CAN 硬件契约；服务层无需包含 HAL 句柄
 */
#ifndef APP_TASK_CAN_PORT_H
#define APP_TASK_CAN_PORT_H
#include <stdbool.h>
#include "app_can_frame.h"
/** @brief 接收启动前注册中断路由。 */
void app_task_can_port_bind(void);
/** @brief 配置 CAN1/CAN2 过滤器并启动接收，返回是否全部成功。 */
bool app_task_can_port_start(void);
/** @brief 非阻塞提交邮箱；失败由 CAN 服务保留待发帧，成功不表示对端收到。 */
bool app_rtos_can_transmit(const app_can_frame_t *frame);
#endif
