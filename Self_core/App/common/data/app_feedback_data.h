/**
 * @file    app_feedback_data.h
 * @brief   电机、功率计与板间角速度的反馈快照契约
 */
#ifndef APP_FEEDBACK_DATA_H
#define APP_FEEDBACK_DATA_H
#include <stdint.h>
#include "app_motor_role.h"
#include "drv_motor.h"
#include "drv_power_measure.h"

/**
 * @brief  CAN 任务发布的电机、功率计和底盘角速度反馈快照
 * @note   云台 motor[0~5]：大 yaw、小 yaw、pitch、左摩擦、拨弹、右摩擦。
 *          底盘 motor[0~3]：右驱动、左驱动、左舵向、右舵向；最后两项未使用。
 *          每个反馈独立保存时间和有效标志，健康监测据此判断数据新鲜度。
 */
typedef struct
{
    drv_motor_data_t motor[GIMBAL_MOTOR_COUNT]; // 最大反馈槽数；槽位含义见 app_motor_role.h。
    uint32_t motor_tick[GIMBAL_MOTOR_COUNT];    // 各电机最近有效反馈的接收时间戳。
    uint8_t motor_valid[GIMBAL_MOTOR_COUNT];    // 各电机是否已收到有效反馈。
    drv_power_data_t power; // 底盘 CAN2 0x212 功率计反馈，含协议电压/电流及 W 功率。
    uint32_t power_tick,    // 最近有效功率计反馈的接收时间戳。
        omega_tick;         // 云台收到的最近有效 CAN1 0x119 时间戳。
    float omega;            // 底盘对地角速度反馈，单位 rad/s。
    uint8_t power_valid,    // 是否已收到有效功率计反馈。
        omega_valid;        // 是否已收到有效底盘角速度反馈。
} app_can_feedback_t;
#endif
