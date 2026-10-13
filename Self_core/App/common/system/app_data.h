#ifndef APP_DATA_H
#define APP_DATA_H
#include "project_cfg.h"
#include "app_sentry_common.h"
#include "app_gimbal_comm.h"
#include "app_chassis_comm.h"
#include "drv_motor.h"
#include "drv_power_measure.h"
#include "drv_dbus.h"
#include "drv_imu.h"
#include "drv_referee.h"

/* Value objects: no HAL/RTOS handle or pointer into a DMA buffer. */
typedef struct { uint32_t std_id; uint8_t bus, dlc, data[8]; uint32_t tick, sequence; } app_can_frame_t;
typedef struct {
    uint32_t id, tick, sequence; uint8_t updated_mask;
    union { app_gimbal_radar_cmd_t radar; app_gimbal_speed_cmd_t delta;
        app_gimbal_angle_cmd_t angle; app_gimbal_shoot_cmd_t shoot;
        app_chassis_speed_cmd_t speed; app_chassis_ackermann_cmd_t ackermann;
        app_chassis_follow_cmd_t follow; } value;
} app_can_ctrl_msg_t;
typedef enum { APP_CAN_GIM_CURRENT, APP_CAN_SHOOT_CURRENT, APP_CAN_CHAS_CURRENT,
    APP_CAN_CHASSIS_CMD, APP_CAN_GIM_FEEDBACK, APP_CAN_CHAS_FEEDBACK } app_ctrl_can_type_t;
typedef struct {
    uint32_t tick, sequence; uint8_t source; app_ctrl_can_type_t type;
    union { int16_t current[4]; app_chassis_speed_cmd_t chassis;
        struct { float yaw_rate, pitch_rate, yaw, pitch; drv_imu_quat_t quat; } gimbal;
        struct { float power, omega; } chassis_state; } value;
} app_ctrl_can_msg_t;
typedef struct app_dbus_snapshot {
    drv_dbus_data_t data; app_gimbal_dbus_input_t input;
    uint32_t tick, sequence; uint8_t valid;
} app_dbus_snapshot_t;
typedef struct { app_gimbal_input_source_t source; uint8_t fire, ready, emergency_stop;
    uint32_t tick, sequence; } app_shoot_command_t;
typedef struct { drv_imu_real_t acc, gyro; drv_imu_quat_t quat; drv_imu_euler_t euler;
    uint32_t tick, sequence; uint8_t valid, sampled, device_valid; } app_imu_snapshot_t;
typedef struct { drv_motor_data_t motor[6]; uint32_t motor_tick[6]; uint8_t motor_valid[6];
    drv_power_data_t power; uint32_t power_tick, omega_tick;
    float omega; uint8_t power_valid, omega_valid; } app_can_feedback_t;
typedef struct { drv_referee_global_t data; uint32_t tick, sequence, updated_mask;
    uint32_t field_tick[20]; uint8_t valid; } app_ref_snapshot_t;
typedef struct { float steer_angle, drive_speed; int8_t drive_rev; } app_sentry_swerve_wheel_t;
typedef struct { app_sentry_swerve_wheel_t wheel_cur[2]; app_sentry_chassis_speed_t cur_speed;
    float cur_power, tar_power, power_scale, battery_voltage, battery_current; } app_sentry_chassis_state_t;
typedef struct { app_dbus_snapshot_t dbus; app_imu_snapshot_t imu; app_can_feedback_t feedback;
    app_gimbal_comm_rx_t can; uint8_t updated_mask; uint32_t tick; } app_gimbal_input_t;
typedef struct { int16_t current[3]; app_shoot_command_t shoot;
    float yaw, pitch, yaw_target, pitch_target, yaw_rate, pitch_rate;
    app_chassis_speed_cmd_t chassis; } app_gimbal_output_t;
typedef struct { app_shoot_command_t command; app_can_feedback_t feedback; } app_shoot_input_t;
typedef struct { int16_t current[3]; float vofa[7]; } app_shoot_output_t;
typedef struct { app_can_feedback_t feedback; app_chassis_comm_rx_t can; } app_chassis_input_t;
typedef struct { int16_t current[4]; app_sentry_chassis_state_t state; app_sentry_chassis_speed_t target;
    float omega; uint8_t feedback_due; } app_chassis_output_t;
typedef struct { app_gimbal_output_t gimbal; app_shoot_output_t shoot; app_chassis_output_t chassis;
    float vofa[8]; uint32_t gimbal_tick, shoot_tick, chassis_tick; } app_ctrl_state_t;
#endif
