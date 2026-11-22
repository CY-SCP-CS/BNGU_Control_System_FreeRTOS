/**
 * @file    app_rtos.h
 * @brief   RTOS 创建、启动状态与诊断接口；不再公开通信对象句柄
 */
#ifndef APP_RTOS_H
#define APP_RTOS_H
#include "app_rtos_kernel.h"
#include "app_rtos_types.h"

/* 创建入口在 Assembly/app_boot.h。业务通信接口见 App/Modules/<功能>/app_task_<功能>.h。
 * 对象仍保留原名字，句柄已改为所属 .c 的 static 私有变量。 */
void app_rtos_count(unsigned index); /**< 任务/合法中断可调用，短临界区增加诊断计数。 */
void app_rtos_count_add(unsigned index, uint32_t delta); /**< 原子累计多次错误，不暴露读改写。 */
void app_rtos_task_complete(
    unsigned task); /**< 一次性任务退出前缓存栈并清空句柄，随后调用 osThreadExit。 */
void app_rtos_heartbeat(unsigned task); /**< 任务维护自己的心跳，独立于设备有效数据时间。 */
void app_rtos_notify_task(unsigned task,
                          uint32_t flags); /**< 通过稳定 TASK_* 索引通知，调用者无需持有句柄。 */
uint32_t app_rtos_init_status(
    void); /**< 返回持久 PORT_READY/SYS_READY/INIT_FAILED 位，自动过滤 CMSIS 错误值。 */
void app_rtos_publish_init(uint32_t flags); /**< 初始化任务向持久事件组发布启动状态。 */
/**
 * @brief  非阻塞释放教学示例二值信号量。
 * @return true=释放成功；false=未启用示例、对象无效或信号量已满。
 * @note   APP_RTOS_ENABLE_EXAMPLES=1 时创建对象；允许任务或合法 RTOS 中断调用。
 */
bool app_rtos_example_sem_release(void);
/**
 * @brief  等待取得教学示例二值信号量。
 * @param[in] timeout 最长等待时间，单位 tick；0 表示不等待。
 * @return true=取得成功；false=示例未启用、对象无效或等待失败。
 * @note   仅任务上下文调用；示例未启用时立即返回，不等待。
 */
bool app_rtos_example_sem_acquire(uint32_t timeout);
extern uint32_t app_rtos_start_tick, app_rtos_init_stack_free;
extern uint8_t app_rtos_init_completed;

#endif
