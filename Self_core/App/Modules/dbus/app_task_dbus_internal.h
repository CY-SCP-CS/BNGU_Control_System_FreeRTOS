/**
 * @file    app_task_dbus.h
 * @brief   遥控输入服务；私有接收队列、快照和 dbus_to_gim_mutex
 */
#ifndef APP_TASK_DBUS_INTERNAL_H
#define APP_TASK_DBUS_INTERNAL_H
#include <stdbool.h>
#include "app_dbus_data.h"

/** @brief USART3 回调立即复制 18 字节；NULL 表示接收错误，禁止保留 DMA 地址。 */
void app_rtos_dbus_receive_isr(const uint8_t bytes[18]);
bool app_rtos_dbus_read(app_dbus_snapshot_t *value);        /**< 任务读取完整副本，超时 0。 */
void app_rtos_dbus_write(const app_dbus_snapshot_t *value); /**< DBUS 任务发布，锁内只复制。 */

/**
 * @brief  DBUS 接收任务入口；完整帧交付和遥控输入快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_dbus_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_dbus_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_dbus_deinit(void);

#endif
