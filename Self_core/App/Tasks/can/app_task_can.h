/**
 * @file    app_task_can.h
 * @brief   CAN 服务语义接口；六个队列和反馈锁由 CAN 模块私有管理
 */
#ifndef APP_TASK_CAN_H
#define APP_TASK_CAN_H
#include <stdbool.h>
#include "app_can_data.h"
#include "app_feedback_data.h"

typedef enum
{
    APP_RTOS_CAN_BUS_1 = 1,
    APP_RTOS_CAN_BUS_2 = 2
} app_rtos_can_bus_t;

typedef enum
{
    APP_RTOS_CAN_TX_COMPLETE,
    APP_RTOS_CAN_IO_ERROR,
    APP_RTOS_CAN_INVALID_FRAME
} app_rtos_can_event_t;

/** @brief ISR 立即复制标准帧到所属 RX 队列；失败计数，不等待。 */
void app_rtos_can_receive_isr(uint8_t bus, uint32_t id, const uint8_t *bytes, uint8_t len,
                              uint32_t tick);
/** @brief 邮箱释放/错误中断通知 CAN 任务，任务管理待发帧。 */
void app_rtos_can_event_isr(app_rtos_can_event_t event);
/** @brief 控制任务非阻塞提交值副本并唤醒 CAN；保留原接口名。 */
void app_rtos_ctrl_send(const app_ctrl_can_msg_t *message);
/** @brief 本板唯一主控制入口取出一条已解包指令；false 表示暂时无消息。 */
bool app_rtos_can_ctrl_receive(app_can_ctrl_msg_t *message);
/** @brief 多读者反馈快照，读取失败保留输出原内容，不在中断调用。 */
bool app_rtos_feedback_read(app_can_feedback_t *value);
/** @brief CAN 任务发布完整反馈，锁内只复制。 */
void app_rtos_feedback_write(const app_can_feedback_t *value);

/**
 * @brief  CAN 通信任务入口；帧交付、控制提交和反馈快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_can_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_can_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_can_deinit(void);

#endif
