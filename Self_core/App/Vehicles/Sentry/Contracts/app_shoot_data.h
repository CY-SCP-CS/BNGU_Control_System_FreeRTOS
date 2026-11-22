/**
 * @file    app_shoot_data.h
 * @brief   发射控制输入、输出和已裁决命令
 */
#ifndef APP_SHOOT_DATA_H
#define APP_SHOOT_DATA_H
#include <stdint.h>
#include "app_dbus_data.h"
#include "app_feedback_data.h"

/**
 * @brief  云台任务裁决后发布给发射任务的控制命令
 * @note   输入可来自 DBUS 或 CAN；发射任务统一使用本命令执行控制。
 *          ready 为云台控制目标对齐后的就绪状态，emergency_stop 为裁决后的急停状态。
 */
typedef struct
{
    app_remote_input_source_t source; // 已裁决的控制源：DBUS、CAN 或急停。
    uint8_t fire,                     // APP_SENTRY_FIRE_* 发射模式值。
        ready,                        // 云台控制已完成初始目标对齐。
        emergency_stop;               // 急停标志，置位时发射任务复位输出。
    uint32_t tick,                    // 本命令的生成时间戳。
        sequence;                     // 云台任务发布命令的序号。
} app_shoot_command_t;

/**
 * @brief  发射单次控制计算使用的命令和电机反馈
 * @note   command 由云台任务裁决，feedback 与云台控制共用同一电机快照来源。
 */
typedef struct
{
    uint32_t tick;               // Current control cycle, kernel tick.
    app_shoot_command_t command; // 已裁决发射输入，含就绪和急停状态。
    app_can_feedback_t feedback; // 电机反馈副本，发射使用索引 3/4/5。
} app_shoot_input_t;

/**
 * @brief  发射控制的输出电流和具名遥测值
 * @note   nominal 为保留的标称波形目标，实际输出仍由 fire/ready/急停决定。
 *          VOFA 通道编号由 app_task_telemetry.c 选择，不进入算法。
 */
typedef struct
{
    int16_t current[3];       /**< 左摩擦轮、拨弹轮、右摩擦轮的电流输出。 */
    float disc_integral;      /**< 拨弹 PID 积分输出 Ki×积分状态，协议 raw，非 A。 */
    float disc_effort_raw;    /**< 拨弹反馈电流协议原值，非 A。 */
    float disc_encoder_raw;   /**< 拨弹 13 位编码器原值。 */
    float disc_nominal;       /**< 拨弹轮标称目标角速度，rad/s。 */
    float disc_speed;         /**< 拨弹轮实测角速度，rad/s。 */
    float friction_nominal;   /**< 左摩擦轮标称目标，右轮为相反数，rad/s。 */
    float friction_l_speed;   /**< 左摩擦轮实测角速度，rad/s。 */
    float friction_r_speed;   /**< 右摩擦轮实测角速度，rad/s。 */
    float friction_r_current; /**< 右摩擦轮反馈电流，原协议值。 */
} app_shoot_output_t;
#endif
