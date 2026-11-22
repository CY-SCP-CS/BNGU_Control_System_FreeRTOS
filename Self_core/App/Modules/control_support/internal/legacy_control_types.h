/** @file legacy_control_types.h
 * @brief 旧控制算法数值尺度；仅算法适配使用，禁止任务和业务消费者包含。
 * @note 长度 mm、线速度 mm/s、IMU acc g/euler deg、电机 encoder/rpm/raw。
 *       PID/限幅/机械常量保留旧值；不可把这些内部数值发布到业务接口。
 */
#ifndef LEGACY_CONTROL_TYPES_H
#define LEGACY_CONTROL_TYPES_H
#include "App/Vehicles/Sentry/Contracts/app_gimbal_data.h"
#include "app_chassis_data.h"
#include "lib_physical.h"

/** @brief 旧电机协议布局；内部恢复编码器和 RPM，避免改变 PID 数值。 */
typedef struct
{
    uint8_t cmd_id;
    int16_t speed;
    uint16_t angle;
    uint8_t temperature;
    int16_t current;
} legacy_motor_t;

/** @brief 旧功率计尺度，bat_v=0.01V/LSB，bat_i=0.01A/LSB，power=W。 */
typedef struct
{
    uint16_t bat_v;
    uint16_t bat_i;
    float power;
} legacy_power_t;

/** @brief 旧欧拉角尺度，pitch/roll/yaw 均为度。 */
typedef struct
{
    float pitch;
    float roll;
    float yaw;
} legacy_euler_t;

typedef struct
{
    int16_t vx;        // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;        // y 方向速度，单位 mm/s，向左为正。
    int16_t vz;        // z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} legacy_chassis_speed_cmd_t;

typedef struct
{
    float speed;       // 车体线速度，单位 mm/s，向前为正。
    float steer_angle; // 转向角，单位 rad，向左为正。
} legacy_chassis_ackermann_cmd_t;

typedef struct
{
    int16_t vx;           // x 方向速度，单位 mm/s，向前为正。
    int16_t vy;           // y 方向速度，单位 mm/s，向左为正。
    int16_t gimbal_angle; // 云台角度 ÷ 0.001 rad，向左为正。
    int16_t custom;       // 协议预留字段。
} legacy_chassis_follow_cmd_t;

typedef struct
{
    legacy_chassis_speed_cmd_t speed;         // 0x111。
    legacy_chassis_ackermann_cmd_t ackermann; // 0x113。
    legacy_chassis_follow_cmd_t follow;       // 0x115。
} legacy_chassis_comm_rx_t;

typedef struct
{
    int16_t vx;        // 云台坐标系 x 方向速度，单位 mm/s。
    int16_t vy;        // 云台坐标系 y 方向速度，单位 mm/s。
    int16_t vz;        // 底盘 z 轴角速度 ÷ 0.001 rad/s，逆时针为正。
    int16_t power_pct; // 功率百分比 × 100，当前由底盘协议保留。
} legacy_gimbal_radar_cmd_t;

typedef struct
{
    float yaw_inc;   // yaw 角度增量，单位 rad，左转为正。
    float pitch_inc; // pitch 角度增量，单位 rad，下转为正。
} legacy_gimbal_speed_cmd_t;

typedef struct
{
    float yaw_abs;   // IMU yaw 绝对角度，单位 rad，左转为正。
    float pitch_abs; // IMU pitch 绝对角度，单位 rad，下转为正。
} legacy_gimbal_angle_cmd_t;

typedef struct
{
    uint8_t shoot_switch; // 摩擦轮和拨弹盘模式：00/FF 组合见通信协议。
    uint8_t retreat;      // 预留控制字节。
} legacy_gimbal_shoot_cmd_t;

typedef struct
{
    legacy_gimbal_radar_cmd_t radar_speed;    // 0x120。
    legacy_gimbal_speed_cmd_t speed_no_shoot; // 0x121。
    legacy_gimbal_angle_cmd_t angle_no_shoot; // 0x123。
    legacy_gimbal_speed_cmd_t speed_shoot;    // 0x125。
    legacy_gimbal_angle_cmd_t angle_shoot;    // 0x127。
    legacy_gimbal_shoot_cmd_t shoot;          // 0x129。
} legacy_gimbal_comm_rx_t;

typedef struct
{
    lib_vec3f_t acc,      // 传感器三轴加速度转换值，单位 g。
        gyro;             // 传感器三轴角速度转换值，单位 rad/s。
    lib_quatf_t quat;     // Mahony 更新后的姿态四元数，无量纲。
    legacy_euler_t euler; // 姿态欧拉角，单位度。
    uint32_t tick,        // 运行样本触发时间戳；初始姿态使用初始化发布时间。
        sequence;         // 运行采样序号，与硬件采样触发对应。
    uint8_t valid,        // 姿态可供控制使用；运行更新时检查 yaw/roll/gyro.z 为有限数。
        sampled,          // 已处理过运行阶段完整样本的标志，序号回绕时仍保持置位。
        device_valid;     // 加速度计和陀螺仪芯片 ID 均匹配的标志。
} legacy_imu_snapshot_t;

typedef struct
{
    legacy_motor_t motor[GIMBAL_MOTOR_COUNT]; // 最大反馈槽数；槽位含义见 app_motor_role.h。
    uint32_t motor_tick[GIMBAL_MOTOR_COUNT];  // 各电机最近有效反馈的接收时间戳。
    uint8_t motor_valid[GIMBAL_MOTOR_COUNT];  // 各电机是否已收到有效反馈。
    legacy_power_t power; // 底盘 CAN2 0x212 功率计反馈，含协议电压/电流及 W 功率。
    uint32_t power_tick,  // 最近有效功率计反馈的接收时间戳。
        omega_tick;       // 云台收到的最近有效 CAN1 0x119 时间戳。
    float omega;          // 底盘对地角速度反馈，单位 rad/s。
    uint8_t power_valid,  // 是否已收到有效功率计反馈。
        omega_valid;      // 是否已收到有效底盘角速度反馈。
} legacy_can_feedback_t;

typedef struct legacy_dbus_snapshot
{
    app_remote_input_t input; // 控制源、归一化摇杆/拨轮输入和发射模式。
    uint32_t tick,            // 本帧接收时间戳。
        sequence;             // 本帧接收序号。
    uint8_t valid;            // DBUS 解包与输入校验通过标志。
} legacy_dbus_snapshot_t;

typedef struct
{
    app_shoot_command_t command;    // 已裁决发射输入，含就绪和急停状态。
    legacy_can_feedback_t feedback; // 电机反馈副本，发射使用索引 3/4/5。
} legacy_shoot_input_t;

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
} legacy_shoot_output_t;

typedef struct
{
    legacy_dbus_snapshot_t dbus;    // 最近取得的完整遥控输入副本。
    legacy_imu_snapshot_t imu;      // 最近取得的完整 IMU 物理量和姿态副本。
    legacy_can_feedback_t feedback; // 本板电机及底盘角速度反馈副本。
    legacy_gimbal_comm_rx_t can;    // 控制任务保存的各类最新 CAN 指令。
    uint8_t fire_allowed;           // Current source has fresh fire authorization.
    uint8_t updated_mask;           // 本周期收到的 APP_GIMBAL_COMM_UPDATE_* 位组合。
    uint32_t tick;                  // 本次云台控制执行的系统时间戳。
} legacy_gimbal_input_t;

typedef struct
{
    int16_t current[3];                 // 电流顺序：大 yaw、小 yaw、pitch。
    app_shoot_command_t shoot;          // 已裁决的发射模式、控制源、就绪和急停状态。
    float yaw,                          // 当前对地 yaw 角度，单位 rad。
        roll,                           /**< 控制机体系横滚角，rad。 */
        pitch,                          // IMU 与 pitch 编码器融合后的角度，单位 rad。
        yaw_target,                     // 当前 yaw 目标角度，单位 rad。
        pitch_target,                   // 当前 pitch 目标角度，单位 rad。
        yaw_rate,                       // 滤波后的 IMU yaw 角速度，单位 rad/s。
        pitch_rate;                     // 滤波后的 pitch 电机角速度，单位 rad/s。
    legacy_chassis_speed_cmd_t chassis; // 准备下发的底盘速度字段，保留协议单位和急停值。
} legacy_gimbal_output_t;

typedef struct
{
    float vx_speed; // x 方向速度，向前为正。
    float vy_speed; // y 方向速度，向左为正。
    float vw_speed; // 绕 z 轴角速度，逆时针为正。
} legacy_sentry_chassis_speed_t;

typedef struct
{
    float steer_angle, // 相对车体 x 轴的转向角，单位 rad。
        drive_speed;   // 带方向符号的轮缘线速度，单位 mm/s。
    int8_t drive_rev;  // 目标驱动方向：1=正向，-1=反向。
} legacy_sentry_swerve_wheel_t;

typedef struct
{
    legacy_sentry_swerve_wheel_t wheel_cur[2]; // 当前舵轮状态：0=左轮，1=右轮。
    legacy_sentry_chassis_speed_t cur_speed;   // 正运动学估算速度；x/y 为 mm/s，w 为 rad/s。
    float cur_power,                           // 低通滤波后的功率计实测功率，单位 W。
        tar_power,                             // 功率闭环目标，单位 W。
        power_scale,                           // 施加到电机电流的缩放系数，范围 [0, 1]。
        battery_voltage,                       // 电池电压，单位 V。
        battery_current;                       // 电池电流，单位 A。
} legacy_sentry_chassis_state_t;

typedef struct
{
    legacy_can_feedback_t feedback; // 本板电机与功率计反馈副本。
    legacy_chassis_comm_rx_t can;   // 控制任务保存的最新速度、阿克曼和跟随参数。
} legacy_chassis_input_t;

typedef struct
{
    int16_t current[4];                   // 右驱动、左驱动、左舵向、右舵向电流；DJI 协议原始值。
    legacy_sentry_chassis_state_t state;  // 当前运动状态、实测功率和功率缩放结果。
    legacy_sentry_chassis_speed_t target; // 限幅后的目标速度；x/y 为 mm/s，w 为 rad/s。
    float omega;                          // 用于 0x119 反馈的滤波车体角速度，单位 rad/s。
} legacy_chassis_output_t;
#endif
