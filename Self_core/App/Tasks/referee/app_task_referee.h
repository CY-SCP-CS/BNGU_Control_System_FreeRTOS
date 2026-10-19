/**
 * @file    app_task_referee.h
 * @brief   裁判服务；接收 FIFO、快照和锁均归本模块所有
 */
#ifndef APP_TASK_REFEREE_H
#define APP_TASK_REFEREE_H
#include <stdbool.h>
#include "app_referee_data.h"

/** @brief ISR 复制有序字节到 SPSC FIFO；(NULL,0) 请求重同步，不使用互斥量。 */
void app_rtos_ref_push_isr(const uint8_t *bytes, uint16_t count);
/** @brief 裁判任务唯一消费者；先处理 reset，再解析返回字节。 */
uint16_t app_rtos_ref_pop(uint8_t *bytes, uint16_t capacity, uint8_t *reset);
bool app_rtos_ref_read(app_ref_snapshot_t *value);        /**< 非阻塞读取完整裁判副本。 */
void app_rtos_ref_write(const app_ref_snapshot_t *value); /**< 裁判任务在批末发布完整快照。 */

/**
 * @brief  裁判接收任务入口；字节流交付、FIFO 读取和裁判快照接口见本头文件
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_ref_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_referee_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_referee_deinit(void);

#endif
