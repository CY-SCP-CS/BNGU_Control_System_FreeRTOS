/**
 * @file    app_ctrl_data.h
 * @brief   供遥测读取的控制汇总契约
 */
#ifndef APP_CTRL_DATA_H
#define APP_CTRL_DATA_H
#include <stdint.h>
#include "app_gimbal_data.h"
#include "app_chassis_data.h"
#include "app_shoot_data.h"

/**
 * @brief  控制任务发布给日志和诊断读取的汇总状态
 * @note   三个子结果分别更新，各自时间戳独立；共享访问由任务层 ctrl_state_mutex 保护。
 *          云台板 vofa[0~6] 为发射波形，vofa[7] 为 IMU roll，单位度。
 *          底盘板 vofa[0~5] 为 vx/vy/w 的目标与实际值，vofa[6/7] 为功率与电流缩放。
 */
typedef struct
{
    app_gimbal_output_t gimbal;   // 最近发布的云台控制结果。
    app_shoot_output_t shoot;     // 最近发布的发射控制结果。
    app_chassis_output_t chassis; // 最近发布的底盘控制结果。
    float vofa[8];                // 当前板型的八通道波形，日志任务按配置打包发送。
    uint32_t gimbal_tick,         // 云台结果最近发布时间戳。
        shoot_tick,               // 发射结果最近发布时间戳。
        chassis_tick;             // 底盘结果最近发布时间戳。
} app_ctrl_state_t;
#endif
