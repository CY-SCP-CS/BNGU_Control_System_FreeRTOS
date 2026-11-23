/** @file app_can_commands.h
 * @brief SI command values and update categories, independent of transport and scheduling.
 */
#ifndef APP_CAN_COMMANDS_H
#define APP_CAN_COMMANDS_H
#include <stdint.h>
#include "app_motion_data.h"

/** 0x120 底盘速度指令；云台在 CAN 控制源下转发给底盘。 */
typedef struct
{
    float vx;        // 云台坐标系 x 方向速度，单位 m/s。
    float vy;        // 云台坐标系 y 方向速度，单位 m/s。
    float vz;        // 底盘 z 轴角速度 ，单位 rad/s，逆时针为正。
    float power_pct; // 功率比例，无量纲，1 表示 100%，当前由底盘协议保留。
} app_gimbal_radar_cmd_t;

/** 0x121、0x125 云台目标角速度指令。 */
typedef struct
{
    float yaw_rate;   // yaw 目标角速度，单位 rad/s，左转为正。
    float pitch_rate; // pitch 目标角速度，单位 rad/s，下转为正。
} app_gimbal_speed_cmd_t;

/** 0x123、0x127 云台绝对角度指令。 */
typedef struct
{
    float yaw_abs;   // IMU yaw 绝对角度，单位 rad，左转为正。
    float pitch_abs; // IMU pitch 绝对角度，单位 rad，下转为正。
} app_gimbal_angle_cmd_t;

/** 0x129 发射机构控制指令的有效字段。 */
typedef struct
{
    uint8_t friction_switch; // 摩擦轮开关：00 关闭，FF 开启。
    uint8_t feed_state;      // 拨弹：00 正转，AA 停转，FF 反转。
} app_gimbal_shoot_cmd_t;

/** 云台板接收并解包后的完整控制数据。 */
typedef struct
{
    app_gimbal_radar_cmd_t radar_speed;    // 0x120。
    app_gimbal_speed_cmd_t speed_no_shoot; // 0x121。
    app_gimbal_angle_cmd_t angle_no_shoot; // 0x123。
    app_gimbal_speed_cmd_t speed_shoot;    // 0x125。
    app_gimbal_angle_cmd_t angle_shoot;    // 0x127。
    app_gimbal_shoot_cmd_t shoot;          // 0x129。
} app_gimbal_comm_rx_t;

/** 接收快照的更新掩码；同一周期可用按位或表示多个新指令。 */
typedef enum
{
    APP_GIMBAL_COMM_UPDATE_RADAR_SPEED = (1U << 0),    // 0x120。
    APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT = (1U << 1), // 0x121。
    APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT = (1U << 2), // 0x123。
    APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT = (1U << 3),    // 0x125。
    APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT = (1U << 4),    // 0x127。
    APP_GIMBAL_COMM_UPDATE_SHOOT = (1U << 5)           // 0x129。
} app_gimbal_comm_update_t;

#endif
