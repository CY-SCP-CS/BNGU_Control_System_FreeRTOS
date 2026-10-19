/**
 * @file    app_sentry_can.c
 * @brief   哨兵 CAN ID、字节序、反馈槽位及控制指令的集中转换
 */
#include "app_sentry_can.h"
#include "app_sentry_common.h"
#include "app_board_config.h"
#include "app_motor_role.h"
#include "lib_math.h"
#include <string.h>
#include <math.h>

app_sentry_can_result_t app_sentry_can_decode(const app_can_frame_t *frame,
                                              app_can_feedback_t *feedback,
                                              app_can_ctrl_msg_t *command, unsigned *motor_index)
{
    if (frame->bus == 2)
    {
        int index = -1;
#if APP_BOARD_HAS_GIMBAL
        /* 云台反馈索引：大 yaw、小 yaw、pitch、左摩擦、拨弹、右摩擦。 */
        const uint16_t ids[GIMBAL_MOTOR_COUNT] = {
            [MOTOR_YAW_L] = SENTRY_CAN_GIMBAL_L_YAW, [MOTOR_YAW_S] = SENTRY_CAN_GIMBAL_S_YAW,
            [MOTOR_PITCH] = SENTRY_CAN_GIMBAL_PITCH, [MOTOR_FRIC_L] = SENTRY_CAN_GIMBAL_F1,
            [MOTOR_DISC] = SENTRY_CAN_GIMBAL_D1,     [MOTOR_FRIC_R] = SENTRY_CAN_GIMBAL_F2};
        for (unsigned i = 0; i < GIMBAL_MOTOR_COUNT; i++)
        {
            if (frame->std_id == ids[i])
            {
                index = (int)i;
            }
        }
#else
        /* 底盘反馈索引：右驱动、左驱动、左舵向、右舵向。 */
        const uint16_t ids[CHASSIS_MOTOR_COUNT] = {[M_DRIVE_R] = SENTRY_CAN_CHASSIS_DRIVE_R,
                                                   [M_DRIVE_L] = SENTRY_CAN_CHASSIS_DRIVE_L,
                                                   [M_STEER_L] = SENTRY_CAN_CHASSIS_STEER_L,
                                                   [M_STEER_R] = SENTRY_CAN_CHASSIS_STEER_R};
        for (unsigned i = 0; i < CHASSIS_MOTOR_COUNT; i++)
        {
            if (frame->std_id == ids[i])
            {
                index = (int)i;
            }
        }
        if (frame->std_id == SENTRY_CAN_CHASSIS_POWER)
        {
            if (frame->dlc != 8)
            {
                return APP_SENTRY_CAN_INVALID;
            }
            drv_power_data_t power;
            drv_power_solve(frame->data, &power);
            if (power.bat_v < 500 || power.bat_v > 6000 || !isfinite(power.power))
            {
                return APP_SENTRY_CAN_INVALID;
            }
            feedback->power = power;
            feedback->power_tick = frame->tick;
            feedback->power_valid = 1;
            return APP_SENTRY_CAN_POWER;
        }
#endif
        if (index >= 0 && frame->dlc == 8)
        {
            drv_motor_data_t data;
            drv_motor_solve_dji(frame->data, &data);
            if (data.angle > 8191)
            {
                return APP_SENTRY_CAN_INVALID;
            }
            feedback->motor[index] = data;
            feedback->motor_tick[index] = frame->tick;
            feedback->motor_valid[index] = 1;
            *motor_index = (unsigned)index;
            return APP_SENTRY_CAN_MOTOR;
        }
        else if (index >= 0)
        {
            return APP_SENTRY_CAN_INVALID;
        }
        return APP_SENTRY_CAN_IGNORED;
    }
    *command = (app_can_ctrl_msg_t){.tick = frame->tick, .sequence = frame->sequence};
#if APP_BOARD_HAS_GIMBAL
    if (frame->std_id == 0x119)
    {
        float omega;
        if (frame->dlc < 4)
        {
            return APP_SENTRY_CAN_INVALID;
        }
        memcpy(&omega, frame->data, 4);
        if (!isfinite(omega) || fabsf(omega) > SENTRY_CHASSIS_VW_TAR_SPEED_MAX * 1.5f)
        {
            return APP_SENTRY_CAN_INVALID;
        }
        feedback->omega = omega;
        feedback->omega_tick = frame->tick;
        feedback->omega_valid = 1;
        return APP_SENTRY_CAN_OMEGA;
    }
    if (frame->std_id != 0x120 && frame->std_id != 0x121 && frame->std_id != 0x125 &&
        frame->std_id != 0x123 && frame->std_id != 0x127 && frame->std_id != 0x129)
    {
        return APP_SENTRY_CAN_IGNORED;
    }
    app_gimbal_comm_rx_t rx;
    if (!app_gimbal_comm_decode(frame->std_id, frame->data, frame->dlc, &rx,
                                &command->updated_mask))
    {
        return APP_SENTRY_CAN_INVALID;
    }
    switch (frame->std_id)
    {
        case 0x120:
            command->type = APP_CAN_CMD_RADAR_SPEED;
            command->value.radar = rx.radar_speed;
            break;
        case 0x121:
            command->type = APP_CAN_CMD_SPEED_NO_SHOOT;
            command->value.delta = rx.speed_no_shoot;
            break;
        case 0x125:
            command->type = APP_CAN_CMD_SPEED_SHOOT;
            command->value.delta = rx.speed_shoot;
            break;
        case 0x123:
            command->type = APP_CAN_CMD_ANGLE_NO_SHOOT;
            command->value.angle = rx.angle_no_shoot;
            break;
        case 0x127:
            command->type = APP_CAN_CMD_ANGLE_SHOOT;
            command->value.angle = rx.angle_shoot;
            break;
        case 0x129:
            command->type = APP_CAN_CMD_SHOOT;
            command->value.shoot = rx.shoot;
            break;
        default:
            return APP_SENTRY_CAN_IGNORED;
    }
#else
    if (frame->std_id != 0x111 && frame->std_id != 0x113 && frame->std_id != 0x115)
    {
        return APP_SENTRY_CAN_IGNORED;
    }
    app_chassis_comm_rx_t rx;
    if (!app_chassis_comm_decode(frame->std_id, frame->data, frame->dlc, &rx,
                                 &command->updated_mask))
    {
        return APP_SENTRY_CAN_INVALID;
    }
    if (frame->std_id == 0x111)
    {
        command->type = APP_CAN_CMD_CHASSIS_SPEED;
        command->value.speed = rx.speed;
    }
    if (frame->std_id == 0x113)
    {
        command->type = APP_CAN_CMD_ACKERMANN;
        command->value.ackermann = rx.ackermann;
    }
    if (frame->std_id == 0x115)
    {
        command->type = APP_CAN_CMD_FOLLOW;
        command->value.follow = rx.follow;
    }
#endif
    return APP_SENTRY_CAN_COMMAND;
}

/** @brief 向调用者的帧数组追加值副本；不持有数组指针，不分配内存。 */
static void append_frame(app_can_frame_t *frames, unsigned *count, uint8_t bus, uint32_t id,
                         const void *bytes, uint8_t dlc, const app_ctrl_can_msg_t *source)
{
    app_can_frame_t *frame = &frames[(*count)++];
    *frame = (app_can_frame_t){
        .bus = bus, .std_id = id, .dlc = dlc, .tick = source->tick, .sequence = source->sequence};
    memcpy(frame->data, bytes, dlc);
}

unsigned app_sentry_can_encode(const app_ctrl_can_msg_t *m,
                               app_can_frame_t frames[APP_SENTRY_CAN_MAX_FRAMES])
{
    uint8_t bytes[8] = {0};
    unsigned count = 0;
    switch (m->type)
    {
        case APP_CAN_GIM_CURRENT:
        case APP_CAN_SHOOT_CURRENT:
            for (unsigned i = 0; i < 3; i++)
            {
                drv_motor_set_dji(bytes, i, m->value.current[i]);
            }
            append_frame(frames, &count, 2, m->type == APP_CAN_GIM_CURRENT ? 0x1FE : 0x200, bytes,
                         8, m);
            break;
        case APP_CAN_CHAS_CURRENT:
            drv_motor_set_dji(bytes, 0, m->value.current[0]);
            drv_motor_set_dji(bytes, 1, m->value.current[1]);
            append_frame(frames, &count, 2, 0x200, bytes, 8, m);
            memset(bytes, 0, 8);
            drv_motor_set_dji(bytes, 0, m->value.current[2]);
            drv_motor_set_dji(bytes, 1, m->value.current[3]);
            append_frame(frames, &count, 2, 0x1FF, bytes, 8, m);
            break;
        case APP_CAN_CHASSIS_CMD:
            append_frame(frames, &count, 1, 0x111, &m->value.chassis, 8, m);
            break;
        case APP_CAN_GIM_FEEDBACK:
        {
            if (!isfinite(m->value.gimbal.yaw_rate) || !isfinite(m->value.gimbal.pitch_rate) ||
                !isfinite(m->value.gimbal.yaw) || !isfinite(m->value.gimbal.pitch) ||
                !isfinite(m->value.gimbal.quat.q0) || !isfinite(m->value.gimbal.quat.q1) ||
                !isfinite(m->value.gimbal.quat.q2) || !isfinite(m->value.gimbal.quat.q3))
            {
                return 0;
            }
            memcpy(bytes, &m->value.gimbal.yaw_rate, 4);
            memcpy(bytes + 4, &m->value.gimbal.pitch_rate, 4);
            append_frame(frames, &count, 1, 0x122, bytes, 8, m);
            memcpy(bytes, &m->value.gimbal.yaw, 4);
            memcpy(bytes + 4, &m->value.gimbal.pitch, 4);
            append_frame(frames, &count, 1, 0x124, bytes, 8, m);
            int16_t q[4] = {(int16_t)(m->value.gimbal.quat.q0 * 30000),
                            (int16_t)(m->value.gimbal.quat.q1 * 30000),
                            (int16_t)(m->value.gimbal.quat.q2 * 30000),
                            (int16_t)(m->value.gimbal.quat.q3 * 30000)};
            append_frame(frames, &count, 1, 0x233, q, 8, m);
            break;
        }
        case APP_CAN_CHAS_FEEDBACK:
        {
            int16_t power = (int16_t)lib_clamp(m->value.chassis_state.power * 100, -32768, 32767);
            memcpy(bytes, &power, 2);
            append_frame(frames, &count, 1, 0x112, bytes, 8, m);
            memset(bytes, 0, 8);
            memcpy(bytes, &m->value.chassis_state.omega, 4);
            append_frame(frames, &count, 1, 0x119, bytes, 8, m);
            break;
        }
        default:
            return 0;
    }
    return count;
}
