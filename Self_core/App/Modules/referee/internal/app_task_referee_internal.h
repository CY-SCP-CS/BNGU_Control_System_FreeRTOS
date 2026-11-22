/**
 * @file    app_task_referee_internal.h
 * @brief   裁判服务；接收 FIFO、快照和锁均归本模块所有
 */
#ifndef APP_TASK_REFEREE_INTERNAL_H
#define APP_TASK_REFEREE_INTERNAL_H
#include "app_task_referee.h"

/**
 * @brief  复制有序裁判字节流到接收 FIFO。
 * @param[in] bytes 至少 count 字节的缓冲，立即复制；NULL 配合 0 请求重同步。
 * @param[in] count 输入长度，单位 B。
 * @return 无返回值；溢出计数并请求重同步。
 * @note   单生产者接收回调调用，不等待，不使用互斥量，不保留 DMA 地址。
 */
void app_rtos_ref_push_isr(const uint8_t *bytes, uint16_t count);
/**
 * @brief  请求裁判接收流和解析器重新同步。
 * @return 无返回值；累计 UART 错误并通知裁判任务。
 * @note   合法 RTOS 中断或设备恢复上下文调用，不等待。
 */
void app_rtos_ref_error_isr(void);

/**
 * @brief  裁判接收任务入口；字节流交付、FIFO 读取和裁判快照接口见本头文件
 * @param[in] argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @return 无返回值；任务持续运行。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_comm_ref_Task(void *argument);

/**
 * @brief  创建本模块通信对象并复位初始状态。
 * @return true=全部创建成功；false=创建失败，由装配统一回滚。
 * @note   仅内核就绪、调度器启动前调用，不支持运行中重置。
 */
bool app_task_referee_init(void);
/**
 * @brief  删除本模块已创建的通信对象。
 * @return 无返回值；空句柄安全跳过。
 * @note   仅启动失败回滚使用，必须先终止所有已创建任务。
 */
void app_task_referee_deinit(void);

#endif
