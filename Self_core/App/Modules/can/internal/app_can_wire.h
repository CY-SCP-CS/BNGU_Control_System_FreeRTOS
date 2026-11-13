/** @file app_can_wire.h
 * @brief CAN 线协议布局，禁止作为业务快照；小端，固定 8 字节，保留原 raw 缩放。
 */
#ifndef APP_CAN_WIRE_H
#define APP_CAN_WIRE_H
#include <stdint.h>

typedef struct
{
    int16_t vx;        // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;        // y 方向速度，单位 mm/s，向左为正。
    int16_t vz;        // z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} wire_chassis_speed_cmd_t;

typedef struct
{
    float speed;       // 车体线速度，单位 mm/s，向前为正。
    float steer_angle; // 转向角，单位 rad，向左为正。
} wire_chassis_ackermann_cmd_t;

typedef struct
{
    int16_t vx;           // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;           // y 方向速度，单位 mm/s，向左为正。
    int16_t gimbal_angle; // 云台角度 ÷ 0.001 rad，向左为正。
    int16_t custom;       // 协议预留字段。
} wire_chassis_follow_cmd_t;

typedef struct
{
    int16_t vx;        // 云台坐标系 x 方向速度，单位 mm/s。
    int16_t vy;        // 云台坐标系 y 方向速度，单位 mm/s。
    int16_t vz;        // 底盘 z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} wire_gimbal_radar_cmd_t;

typedef struct
{
    float yaw_inc;   // yaw 角度增量，单位 rad，左转为正。
    float pitch_inc; // pitch 角度增量，单位 rad，下转为正。
} wire_gimbal_speed_cmd_t;

typedef struct
{
    float yaw_abs;   // IMU yaw 绝对角度，单位 rad，左转为正。
    float pitch_abs; // IMU pitch 绝对角度，单位 rad，下转为正。
} wire_gimbal_angle_cmd_t;

typedef struct
{
    uint8_t shoot_switch; // 摩擦轮和拨弹盘模式：00/FF 组合见通信协议。
    uint8_t retreat;      // 预留控制字节。
} wire_gimbal_shoot_cmd_t;
#endif
