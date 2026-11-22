/**
 * @file    app_vehicle_can.h
 * @brief   车型 CAN 协议编解码；纯值转换，不发送、不操作队列或硬件
 */
#ifndef APP_VEHICLE_CAN_H
#define APP_VEHICLE_CAN_H
#include "App/Modules/can/app_can_data.h"
#include "app_feedback_data.h"

typedef enum
{
    APP_VEHICLE_CAN_IGNORED,
    APP_VEHICLE_CAN_INVALID,
    APP_VEHICLE_CAN_MOTOR,
    APP_VEHICLE_CAN_POWER,
    APP_VEHICLE_CAN_OMEGA,
    APP_VEHICLE_CAN_COMMAND
} app_vehicle_can_result_t;

/** @brief 单帧解包；返回更新类别，只有 MOTOR 返回时 motor_index 有效。 */
app_vehicle_can_result_t app_vehicle_can_decode(const app_can_frame_t *frame,
                                                app_can_feedback_t *feedback,
                                                app_can_ctrl_msg_t *command, unsigned *motor_index);
/* 一个控制结果最多编码为四帧（云台反馈）。调用者提供完整数组，返回实际帧数；0 表示非法结果。 */
#define APP_VEHICLE_CAN_MAX_FRAMES 4U
unsigned app_vehicle_can_encode(const app_ctrl_can_msg_t *message,
                                app_can_frame_t frames[APP_VEHICLE_CAN_MAX_FRAMES]);
#endif
