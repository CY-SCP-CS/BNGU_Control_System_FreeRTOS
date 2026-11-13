/** @file app_task_can.h
 * @brief can 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_CAN_H
#define APP_TASK_CAN_H
#include <stdbool.h>
#include "app_can_data.h"
#include "app_feedback_data.h"
void app_rtos_ctrl_send(const app_ctrl_can_msg_t *message);
bool app_rtos_can_ctrl_receive(app_can_ctrl_msg_t *message);
/**
 * @brief  读取一份完整的 CAN 电机/功率计/底盘角速度反馈快照
 * @param[in] value            输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由控制及诊断读取者在任务上下文调用；使用 can_feedback_mutex，获取锁超时为 0。
 */
bool app_rtos_feedback_read(app_can_feedback_t *value);
#endif
