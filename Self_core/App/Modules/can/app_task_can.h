/** @file app_task_can.h
 * @brief can 跨模块服务；无设备实例、任务入口或通信句柄。 */
#ifndef APP_TASK_CAN_H
#define APP_TASK_CAN_H
#include <stdbool.h>
#include "App/Modules/can/app_can_data.h"
#include "app_feedback_data.h"
/**
 * @brief  非阻塞提交当前控制结果并唤醒 CAN 任务。
 * @param[in] message 非 NULL 结果，立即复制；不保留地址，字段单位见业务契约。
 * @return 无返回值；无效参数或队列满时累计 COUNT_CTRL_CAN_DROP。
 * @note   控制任务调用；周期状态由下一周期重新提交，单次事件不可据此假设已交付。
 */
void app_rtos_ctrl_send(const app_ctrl_can_msg_t *message);
/**
 * @brief  非阻塞取出一条已解包的 CAN 指令。
 * @param[out] message 非 NULL 输出，容量为一份指令；失败时保留原值。
 * @return true=取出一条指令；false=参数无效、无消息或队列不可用。
 * @note   仅本板主控制任务作为单消费者调用，不向多个控制任务广播。
 */
bool app_rtos_can_ctrl_receive(app_can_ctrl_msg_t *message);
/**
 * @brief  读取一份完整的 CAN 电机/功率计/底盘角速度反馈快照
 * @param[out] value           输出存储，失败时保留原内容
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   由控制及诊断读取者在任务上下文调用；使用 can_feedback_mutex，获取锁超时为 0。
 */
bool app_rtos_feedback_read(app_can_feedback_t *value);

/* Service-owned resource capacities and per-cycle budgets. */
#define APP_RTOS_CAN_RX_QUEUE_DEPTH 32U
#define APP_RTOS_CAN_TX_QUEUE_DEPTH 16U
#define APP_RTOS_CTRL_QUEUE_DEPTH 16U
#define APP_RTOS_CAN_DATA_SIZE 8U
#define APP_RTOS_CAN_RX_BUDGET 8U
#define APP_RTOS_CAN_CTRL_BUDGET 8U
#define APP_RTOS_CAN_PENDING_DEPTH 16U
#define APP_RTOS_CAN_TX_BUDGET 16U
#define APP_RTOS_CTRL_INPUT_BUDGET 16U
#endif
