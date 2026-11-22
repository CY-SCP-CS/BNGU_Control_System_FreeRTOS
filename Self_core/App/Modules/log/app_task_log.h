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

/**
 * @brief  按等级/模式过滤后，非阻塞投递定长事件。
 * @param[in] source 来源任务编号 TASK_*。
 * @param[in] level 日志等级，1=信息、2=告警、3=错误。
 * @param[in] code 稳定事件码，语义由事件定义决定。
 * @param[in] a 第一个整数参数，单位由事件码约定，立即复制。
 * @param[in] b 第二个整数参数，单位由事件码约定，立即复制。
 * @return 无返回值；模式/等级过滤直接返回，队列满累计 COUNT_LOG_DROP。
 * @note   TEXT 由 APP_LOG_MIN_LEVEL 选择等级；VOFA 过滤文本事件但不影响健康状态和计数。
 *          队列满累计 COUNT_LOG_DROP；调用方不格式化、不等待串口。
 */
void app_rtos_log(uint8_t source, uint8_t level, uint16_t code, int32_t a, int32_t b);

#endif
