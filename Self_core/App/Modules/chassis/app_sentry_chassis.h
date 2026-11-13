/**
 * @file    app_sentry_chassis.h
 * @brief   哨兵双舵轮底盘算法接口：输入指令与反馈快照，输出电流和遥测。
 */
#ifndef APP_SENTRY_CHASSIS_H
#define APP_SENTRY_CHASSIS_H
#include "app_chassis_data.h"
#include "lib_pid.h"

/**
 * @brief  初始化底盘算法状态、PID、功率和角速度滤波器。
 * @note   由初始化任务调用一次，控制开始后状态由 ctrl_chas_Task 独占。
 * @note   功率目标沿用 SENTRY_CHASSIS_TAR_POWER=100 W，不在此处启动硬件。
 */
void app_sentry_chassis_init(void);
/**
 * @brief  完成一次 1 ms 双舵轮计算并输出本周期结果。
 * @param  input        完整底盘指令及 CAN 反馈副本；不可为 NULL。
 * @param  output       输出：右/左驱动、左/右转向电流，底盘状态、目标速度和反馈发送标记；不可为 NULL。
 * @note   由 ctrl_chas_Task 周期调用，步长来自 APP_CONTROL_CHAS_DT_S，不调用 HAL 或 RTOS。
 * @note   feedback_due 每 5 次计算置位一次，任务据此提交 200 Hz 板间反馈；实际发送由 CAN 任务完成。
 */
void app_sentry_chassis_ctrl(const app_chassis_input_t *, app_chassis_output_t *);
#endif
