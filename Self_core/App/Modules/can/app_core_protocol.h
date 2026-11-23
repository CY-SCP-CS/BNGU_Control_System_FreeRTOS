/** Shared core-bus auxiliary formats. No assumed IMU bus, polling or RFID mapping. */
#ifndef APP_CORE_PROTOCOL_H
#define APP_CORE_PROTOCOL_H
#include "app_can_data.h"
#include <stdbool.h>

typedef struct
{
    int16_t heat_remaining;
    uint8_t stage, hurt_type, warning_level, warning_robot, flags, recovery_pct, heat_ratio;
    uint16_t remaining_s, hp, max_hp, ally_1_hp, ally_3_hp;
    uint8_t robot_id, valid_mask;
    uint32_t tick[4];
} app_core_referee_t;

typedef struct
{
    float pitch, yaw, roll;
} app_core_euler_t;

bool app_core_referee_decode(const app_can_frame_t *frame, app_core_referee_t *state);
bool app_core_referee_encode(uint32_t id, const app_core_referee_t *state, app_can_frame_t *frame);
bool app_core_imu_read_request(uint8_t bus, uint8_t rid, app_can_frame_t *frame);
bool app_core_imu_euler_decode(const app_can_frame_t *frame, app_core_euler_t *euler);
bool app_core_quaternion_decode(const app_can_frame_t *frame, float quaternion[4]);
#endif
