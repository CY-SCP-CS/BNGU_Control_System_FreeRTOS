/**
 * @file    app_chassis_data.h
 * @brief   底盘控制输入、输出与运动状态
 */
#ifndef APP_CHASSIS_DATA_H
#define APP_CHASSIS_DATA_H
#include <stdint.h>
#include "app_feedback_data.h"
#include "app_motion_data.h"

/** 车体坐标系速度，x/y 单位 m/s，w 单位 rad/s。 */
typedef struct
{
    float vx_speed; // x 方向速度，向前为正。
    float vy_speed; // y 方向速度，向左为正。
    float vw_speed; // 绕 z 轴角速度，逆时针为正。
} app_chassis_velocity_t;

/**
 * @brief  单个舵轮的转向角、轮缘速度与驱动方向
 * @note   目标舵轮使用 drive_rev 表示最短转向选择的驱动符号；当前反馈状态中置为 1。
 */
typedef struct
{
    float steer_angle, // 相对车体 x 轴的转向角，单位 rad。
        drive_speed;   // 带方向符号的轮缘线速度，单位 m/s。
    int8_t drive_rev;  // 目标驱动方向：1=正向，-1=反向。
} app_swerve_wheel_state_t;

/**
 * @brief  底盘控制发布的运动状态和功率闭环状态
 * @note   当前使用功率计实测功率闭环，tar_power 保持原固定 100 W 目标。
 */
typedef struct
{
    app_swerve_wheel_state_t wheel_cur[2]; // 当前舵轮状态：0=左轮，1=右轮。
    app_chassis_velocity_t cur_speed;      // 正运动学估算速度；x/y 为 m/s，w 为 rad/s。
    float cur_power,                       // 低通滤波后的功率计实测功率，单位 W。
        tar_power,                         // 功率闭环目标，单位 W。
        power_scale,                       // 施加到电机电流的缩放系数，范围 [0, 1]。
        battery_voltage,                   // 电池电压，单位 V。
        battery_current;                   // 电池电流，单位 A。
} app_chassis_state_t;

/**
 * @brief  底盘单次控制计算使用的电机反馈与板间指令
 * @note   当前控制使用 0x111 速度指令；0x113/115 继续解析存储，供后续模式扩展。
 */
typedef struct
{
    app_can_feedback_t feedback; // 本板电机与功率计反馈副本。
    uint32_t tick, command_tick; // Current cycle and last SPEED command receive tick.
    uint8_t command_valid;       // ACKERMANN/FOLLOW never renew the SPEED lease.
    app_chassis_comm_rx_t can;   // 控制任务保存的最新速度、阿克曼和跟随参数。
} app_chassis_input_t;

/**
 * @brief  底盘单次控制计算产生的电流、运动状态与遥测数据
 * @note   反馈发送节拍由底盘任务决定，算法只计算当前状态。
 */
typedef struct
{
    int16_t current[4];            // 右驱动、左驱动、左舵向、右舵向电流；DJI 协议原始值。
    app_chassis_state_t state;     // 当前运动状态、实测功率和功率缩放结果。
    app_chassis_velocity_t target; // 限幅后的目标速度；x/y 为 m/s，w 为 rad/s。
    float omega;                   // 用于 0x119 反馈的滤波车体角速度，单位 rad/s。
} app_chassis_output_t;
#endif
