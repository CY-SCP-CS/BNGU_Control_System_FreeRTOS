/**
 * @file    app_rtos_config.h
 * @brief   调度、容量与处理预算；修改数值时评估延迟、带宽和 SRAM 预算
 */
#ifndef APP_RTOS_CONFIG_H
#define APP_RTOS_CONFIG_H
#include "app_control_config.h"

/* 以下周期和超时以 tick 表示；app_rtos.c 强制要求内核频率 1000 Hz。 */
#define APP_RTOS_GIM_PERIOD APP_CONTROL_GIM_PERIOD_MS
#define APP_RTOS_CHAS_PERIOD APP_CONTROL_CHAS_PERIOD_MS
#define APP_RTOS_SHOOT_PERIOD APP_CONTROL_SHOOT_PERIOD_MS
#define APP_RTOS_HEALTH_PERIOD 20U
#define APP_RTOS_VOFA_PERIOD 5U
#define APP_RTOS_EVENT_WAIT 50U
#define APP_RTOS_IMU_POLL_WAIT 1U
#define APP_RTOS_START_MARGIN 2U
#define APP_RTOS_GIM_FEEDBACK_DIVIDER (APP_CONTROL_FEEDBACK_PERIOD_MS / APP_CONTROL_GIM_PERIOD_MS)
#define APP_RTOS_CHAS_FEEDBACK_DIVIDER (APP_CONTROL_FEEDBACK_PERIOD_MS / APP_CONTROL_CHAS_PERIOD_MS)
#define APP_RTOS_TASK_ALIVE_TIMEOUT 100U
#define APP_RTOS_HEALTH_REPORT_PERIOD 1000U  //错误计数增量的汇总周期
#define APP_RTOS_MEMORY_SAMPLE_PERIOD 1000U  //堆/栈采样；在线和心跳仍按 20 ms 检查
#define APP_RTOS_HEALTH_REMIND_PERIOD 10000U //持续故障提醒；状态变化不等待此周期
#define APP_RTOS_LOG_MEMORY_PERIOD 5000U     //正常内存摘要周期

/* 日志只保留固定事件队列和一个发送者，不在生产者中格式化。
 * 详细栈表在初始化结束后输出一次；置 1 可在每次摘要后追加详细表。 */
#ifndef APP_LOG_MIN_LEVEL
#define APP_LOG_MIN_LEVEL 1U //1=信息及以上，2=告警及以上，3=仅错误
#endif
#ifndef APP_LOG_MEMORY_DETAIL
#define APP_LOG_MEMORY_DETAIL 0
#endif
#if APP_LOG_MIN_LEVEL < 1 || APP_LOG_MIN_LEVEL > 3
#error "APP_LOG_MIN_LEVEL must be 1, 2 or 3"
#endif
#if APP_RTOS_MEMORY_SAMPLE_PERIOD == 0 || APP_RTOS_HEALTH_REMIND_PERIOD == 0 ||                    \
    APP_RTOS_LOG_MEMORY_PERIOD == 0
#error "Memory sampling and diagnostic report periods must be positive"
#endif
#define APP_RTOS_STACK_MIN_BYTES 128U
#define APP_RTOS_STACK_RESERVE_DIVISOR 5U
#define APP_RTOS_HEAP_MIN_BYTES 1024U

/* 容量直接用于创建对象；不再存在头文件写一个值、创建代码另写常量的情况。 */
#define APP_RTOS_CAN_RX_QUEUE_DEPTH 32U
#define APP_RTOS_CAN_TX_QUEUE_DEPTH 16U
#define APP_RTOS_CTRL_QUEUE_DEPTH 16U
#define APP_RTOS_LOG_QUEUE_DEPTH 16U
#define APP_RTOS_DBUS_QUEUE_DEPTH 4U
#define APP_RTOS_IMU_QUEUE_DEPTH 4U
#define APP_RTOS_CAN_DATA_SIZE 8U
#define APP_RTOS_CAN_RX_BUDGET 8U
#define APP_RTOS_CAN_CTRL_BUDGET 8U
#define APP_RTOS_CAN_PENDING_DEPTH 16U
#define APP_RTOS_REF_FIFO_SIZE 1024U
#define APP_RTOS_REF_CHUNK_SIZE 128U
#define APP_RTOS_REF_BATCH_CHUNKS 8U
#ifndef APP_LOG_VOFA
#define APP_LOG_VOFA 0
#endif

#endif

/* Work budgets are independent of storage capacity. */
#define APP_RTOS_CAN_TX_BUDGET 16U
#define APP_RTOS_CTRL_INPUT_BUDGET 16U
#define APP_RTOS_DBUS_PARSE_BUDGET 4U
#define APP_RTOS_IMU_PARSE_BUDGET 4U
