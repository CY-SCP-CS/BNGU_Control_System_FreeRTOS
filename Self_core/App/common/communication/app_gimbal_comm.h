/**
 * @file    app_gimbal_comm.h
 * @brief   云台 CAN1 板间通信协议：接收控制指令，发送云台状态反馈
 * @note    云台坐标系：前为 x、左为 y、上为 z；yaw 左转为正，pitch 下转为正。
 *          多字节数据按小端序编码；STM32F4 可直接复制整数和 float 字节。
 */
#ifndef APP_GIMBAL_COMM_H
#define APP_GIMBAL_COMM_H

#include "lib_typedef.h"

#define APP_GIMBAL_CAN_ID_RADAR_SPEED          0x120U  // 雷达/上位机底盘速度指令。
#define APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT       0x121U  // 无射击云台增量角度指令。
#define APP_GIMBAL_CAN_ID_SPEED_FEEDBACK       0x122U  // 云台角速度反馈。
#define APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT       0x123U  // 无射击云台绝对角度指令。
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK       0x124U  // 云台绝对角度反馈。
#define APP_GIMBAL_CAN_ID_SPEED_SHOOT          0x125U  // 带射击云台增量角度指令。
#define APP_GIMBAL_CAN_ID_SHOOT_FEEDBACK       0x126U  // 发射机构状态反馈。
#define APP_GIMBAL_CAN_ID_ANGLE_SHOOT          0x127U  // 带射击云台绝对角度指令。
#define APP_GIMBAL_CAN_ID_SHOOT_CMD            0x129U  // 发射机构控制指令，DLC 为 4。
#define APP_GIMBAL_CAN_ID_ANGLE_FEEDBACK_V2    0x130U  // 预留的角度反馈 V2。
#define APP_GIMBAL_CAN_ID_IMU_QUATERNION       0x233U  // IMU 四元数反馈。
#define APP_GIMBAL_CAN_ID_DM_IMU_REQUEST       0x31U   // 预留的达妙 IMU 请求。
#define APP_GIMBAL_CAN_ID_DM_IMU_RESPONSE      0x39U   // 预留的达妙 IMU 响应。
#define APP_GIMBAL_ANGLE_SHOOT_TARGET_DETECTED 0xFFFFU // 角度射击协议的目标检测标志。

/** 0x120 底盘速度指令；云台在 CAN 控制源下转发给底盘。 */
typedef struct {
    int16_t vx;        // 云台坐标系 x 方向速度，单位 mm/s。
    int16_t vy;        // 云台坐标系 y 方向速度，单位 mm/s。
    int16_t vz;        // 底盘 z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} app_gimbal_radar_cmd_t;

/** 0x121、0x125 云台增量角度指令。 */
typedef struct {
    float yaw_inc;     // yaw 角度增量，单位 rad，左转为正。
    float pitch_inc;   // pitch 角度增量，单位 rad，下转为正。
} app_gimbal_speed_cmd_t;

/** 0x123、0x127 云台绝对角度指令。 */
typedef struct {
    float yaw_abs;     // IMU yaw 绝对角度，单位 rad，左转为正。
    float pitch_abs;   // IMU pitch 绝对角度，单位 rad，下转为正。
} app_gimbal_angle_cmd_t;

/** 0x129 发射机构控制指令的有效字段。 */
typedef struct {
    uint8_t shoot_switch; // 摩擦轮和拨弹盘模式：00/FF 组合见通信协议。
    uint8_t retreat;      // 预留控制字节。
} app_gimbal_shoot_cmd_t;

/** 云台板接收并解包后的完整控制数据。 */
typedef struct {
    app_gimbal_radar_cmd_t radar_speed;    // 0x120。
    app_gimbal_speed_cmd_t speed_no_shoot; // 0x121。
    app_gimbal_angle_cmd_t angle_no_shoot; // 0x123。
    app_gimbal_speed_cmd_t speed_shoot;    // 0x125。
    app_gimbal_angle_cmd_t angle_shoot;    // 0x127。
    app_gimbal_shoot_cmd_t shoot;          // 0x129。
} app_gimbal_comm_rx_t;

/** S2 三挡输入源；遥控器的物理挡位从上到下为 1、3、2。 */
typedef enum {
    APP_GIMBAL_INPUT_DBUS  = 1U, // S2 上挡：使用遥控器。
    APP_GIMBAL_INPUT_CAN   = 3U, // S2 中挡：使用 CAN 上位机指令。
    APP_GIMBAL_INPUT_ESTOP = 2U  // S2 下挡：急停。
} app_gimbal_input_source_t;

/** 遥控器采样值，通信层只归一化，不乘各机构的速度上限。 */
typedef struct {
    app_gimbal_input_source_t source;     // 当前 S2 选择的控制源。
    float chassis_vx_norm;                 // ch0 归一化后的底盘 x 指令，范围 [-1, 1]。
    float chassis_vy_norm;                 // ch1 归一化后的底盘 y 指令，范围 [-1, 1]。
    float chassis_omega_norm;              // 拨轮归一化后的底盘角速度指令，范围 [-1, 1]。
    float yaw_rate_norm;                   // ch2 归一化后的 yaw 角速度指令，范围 [-1, 1]。
    float pitch_rate_norm;                 // ch3 归一化后的 pitch 角速度指令，范围 [-1, 1]。
    uint8_t launcher_mode;                 // S1 三挡发射机构控制字。
} app_gimbal_dbus_input_t;

/** 接收快照的更新掩码；同一周期可用按位或表示多个新指令。 */
typedef enum {
    APP_GIMBAL_COMM_UPDATE_RADAR_SPEED    = (1U << 0), // 0x120。
    APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT = (1U << 1), // 0x121。
    APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT = (1U << 2), // 0x123。
    APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT    = (1U << 3), // 0x125。
    APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT    = (1U << 4), // 0x127。
    APP_GIMBAL_COMM_UPDATE_SHOOT          = (1U << 5)  // 0x129。
} app_gimbal_comm_update_t;

/* Pure protocol helpers; state belongs to the consuming control task. */
uint8_t app_gimbal_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
    app_gimbal_comm_rx_t *rx, uint8_t *updated_mask);
struct app_dbus_snapshot;
uint8_t app_gimbal_comm_dbus_rx(const struct app_dbus_snapshot *snapshot, app_gimbal_dbus_input_t *input);

#endif
