/**
 * @file    app_feedback_data.h
 * @brief   电机、功率计与板间角速度的反馈快照契约
 */
#ifndef APP_FEEDBACK_DATA_H
#define APP_FEEDBACK_DATA_H
#include <stdint.h>
#include "app_motor_role.h"

/** @brief 已验证的电机物理反馈；CAN 任务写，互斥快照读，机构方向由算法解释。 */
typedef struct
{
    float angle_rad;     /**< [0,2π) 编码器角度，设备零位，不含机构偏置。 */
    float speed_rad_s;   /**< 轴转速，rad/s，保持设备正方向。 */
    int16_t effort_raw;  /**< 协议反馈控制量，[-32768,32767]，未确认 A 换算。 */
    float temperature_c; /**< 设备温度，°C，工程约定。 */
} app_motor_feedback_t;

/** @brief 功率计物理反馈；电流符号沿用原无符号协议。 */
typedef struct
{
    float voltage_v; /**< V，原有效范围 [5,60]。 */
    float current_a; /**< A，原 0.01A/LSB。 */
    float power_w;   /**< W，沿用协议 float32。 */
} app_power_feedback_t;

/**
 * @brief  CAN 任务发布的电机、功率计和底盘角速度反馈快照
 * @note   云台 motor[0~5]：大 yaw、小 yaw、pitch、左摩擦、拨弹、右摩擦。
 *          底盘 motor[0~3]：右驱动、左驱动、左舵向、右舵向；最后两项未使用。导航车 motor[0..3] 为 CAN2 ID 1/2/3/4（FR/RL/RR/FL）。
 *          每个反馈独立保存时间和有效标志，健康监测据此判断数据新鲜度。
 */
typedef struct
{
    app_motor_feedback_t motor[GIMBAL_MOTOR_COUNT]; // 最大反馈槽数；槽位含义见 app_motor_role.h。
    uint32_t motor_tick[GIMBAL_MOTOR_COUNT];        // 各电机最近有效反馈的接收时间戳。
    uint8_t motor_valid[GIMBAL_MOTOR_COUNT];        // 各电机是否已收到有效反馈。
    app_power_feedback_t power; // 底盘 CAN2 0x212 功率计反馈，含协议电压/电流及 W 功率。
    uint32_t power_tick,        // 最近有效功率计反馈的接收时间戳。
        omega_tick;             // 云台收到的最近有效 CAN1 0x119 时间戳。
    float omega;                // 底盘对地角速度反馈，单位 rad/s。
    uint8_t power_valid,        // 是否已收到有效功率计反馈。
        omega_valid;            // 是否已收到有效底盘角速度反馈。
} app_can_feedback_t;
#endif
