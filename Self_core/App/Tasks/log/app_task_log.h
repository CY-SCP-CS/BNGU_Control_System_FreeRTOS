/**
 * @file    app_task_log.h
 * @brief   日志服务；对外只开放事件提交，UART1 由日志任务独占
 */
#ifndef APP_TASK_LOG_H
#define APP_TASK_LOG_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 固定长度日志事件；不在控制任务中格式化文本。 */
typedef struct
{
    uint32_t tick;  //事件时间，单位内核 tick
    uint16_t code;  //事件码，具体意义由事件类型决定
    uint8_t source, //来源任务编号 TASK_*
        level;      //1=信息，2=告警，3=错误
    int32_t value0, //事件整数参数 0
        value1;     //事件整数参数 1
} app_rtos_log_msg_t;

/** @brief 非阻塞投递定长事件；队列满累计 COUNT_LOG_DROP，控制任务不格式化。 */
void app_rtos_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b);

/**
 * @brief  系统日志任务入口；日志提交接口见本头文件，UART 发送由任务独占
 * @param  argument CMSIS-RTOS2 任务入口参数，当前创建时传 NULL。
 * @note   由任务表注册并由调度器调用，业务代码不直接调用任务入口。
 */
void Start_sys_log_Task(void *argument);

/** @brief 调度器启动前创建本模块的对象与初始数据。 */
bool app_task_log_init(void);
/** @brief 仅启动失败回滚时调用，调用前确保所属任务未运行。 */
void app_task_log_deinit(void);

#endif
