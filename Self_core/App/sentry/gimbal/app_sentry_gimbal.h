#ifndef APP_SENTRY_GIMBAL_H
#define APP_SENTRY_GIMBAL_H
#include "app_data.h"
#include "lib_pid.h"
typedef struct {
    uint8_t mode;                    // app_sentry_gimbal_mode_t。
    float yaw_tar_speed;             // yaw 目标角速度，rad/s。
    float pitch_tar_speed;           // pitch 目标角速度，rad/s。
    float yaw_tar_angle;             // yaw 目标角度，rad。
    float pitch_tar_angle;           // pitch 目标角度，rad。
    float yaw_tar_angle_delta;       // yaw 目标角度增量，rad。
    float pitch_tar_angle_delta;     // pitch 目标角度增量，rad。
    uint8_t fire;                    // app_sentry_fire_mode_t。
} app_sentry_gimbal_cmd_t;
void app_sentry_gimbal_init(void);
void app_sentry_gimbal_ctrl_1khz(const app_gimbal_input_t *input, app_gimbal_output_t *output);
#endif
