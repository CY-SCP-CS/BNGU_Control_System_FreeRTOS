/**
 * @file    app_task_dbus_internal.h
 * @brief   遥控输入服务；私有接收队列、快照和 dbus_to_gim_mutex
 */
#ifndef APP_TASK_DBUS_INTERNAL_H
#define APP_TASK_DBUS_INTERNAL_H
#include "app_task_dbus.h"

/**
 * @brief  复制一份完整 DBUS 帧或报告接收错误。
 * @param[in] bytes 18 B 帧，立即复制；NULL 表示接收错误。
 * @return 无返回值；错误或队列满时累计诊断计数。
 * @note   合法 RTOS 中断或设备恢复上下文调用，不等待，不保留 DMA 地址。
 */
void app_rtos_dbus_receive_isr(const uint8_t bytes[18]);

/**
 * @brief  DBUS 接收任务入口；完整帧交付和遥控输入快照接口见本头文件
 * @param[in] argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @return 无返回值；任务持续运行。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_dbus_Task(void *argument);

/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=全部创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用，不支持运行中重置。
 */
bool app_task_dbus_init(void);
/**
 * @brief  删除本模块已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚使用，必须先终止所有已创建任务。
 */
void app_task_dbus_deinit(void);

#endif
