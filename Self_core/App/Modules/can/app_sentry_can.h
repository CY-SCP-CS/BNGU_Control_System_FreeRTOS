/**
 * @file    app_sentry_can.h
 * @brief   哨兵 CAN 协议编解码；纯值转换，不发送、不操作队列或硬件
 */
#ifndef APP_SENTRY_CAN_H
#define APP_SENTRY_CAN_H
#include "app_can_data.h"
#include "app_feedback_data.h"

typedef enum
{
    APP_SENTRY_CAN_IGNORED,
    APP_SENTRY_CAN_INVALID,
    APP_SENTRY_CAN_MOTOR,
    APP_SENTRY_CAN_POWER,
    APP_SENTRY_CAN_OMEGA,
    APP_SENTRY_CAN_COMMAND
} app_sentry_can_result_t;

/** @brief 单帧解包；返回更新类别，只有 MOTOR 返回时 motor_index 有效。 */
app_sentry_can_result_t app_sentry_can_decode(const app_can_frame_t *frame,
                                              app_can_feedback_t *feedback,
                                              app_can_ctrl_msg_t *command, unsigned *motor_index);
/* 一个控制结果最多编码为四帧（云台反馈）。调用者提供完整数组，返回实际帧数；0 表示非法结果。 */
#define APP_SENTRY_CAN_MAX_FRAMES 4U
unsigned app_sentry_can_encode(const app_ctrl_can_msg_t *message,
                               app_can_frame_t frames[APP_SENTRY_CAN_MAX_FRAMES]);
#endif
