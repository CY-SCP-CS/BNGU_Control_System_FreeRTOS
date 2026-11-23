#ifndef APP_MOTION_DATA_H
#define APP_MOTION_DATA_H
#include <stdint.h>

/** 0x111 底盘速度指令。 */
typedef struct
{
    float vx;        // x 方向速度，单位 m/s，向前为正。
    float vy;        // y 方向速度，单位 m/s，向左为正。
    float vz;        // z 轴角速度 ，单位 rad/s，逆时针为正。
    float power_pct; // 功率比例，无量纲，1 表示 100%，当前由底盘协议保留。
} app_chassis_speed_cmd_t;

/** 0x113 阿克曼底盘指令。 */
typedef struct
{
    float speed;       // 车体线速度，单位 m/s，向前为正。
    float steer_angle; // 转向角，单位 rad，向左为正。
} app_chassis_ackermann_cmd_t;

/** 0x115 跟随模式指令。 */
typedef struct
{
    float vx;           // x 方向速度，单位 m/s，向前为正。
    float vy;           // y 方向速度，单位 m/s，向左为正。
    float gimbal_angle; // 云台角度 ，单位 rad，向左为正。
    float wz;           // rad/s, core protocol 0x115.
} app_chassis_follow_cmd_t;

/** 底盘板接收并解包后的全部控制数据。 */
typedef struct
{
    app_chassis_speed_cmd_t speed;         // 0x111。
    app_chassis_ackermann_cmd_t ackermann; // 0x113。
    app_chassis_follow_cmd_t follow;       // 0x115。
} app_chassis_comm_rx_t;

#endif
