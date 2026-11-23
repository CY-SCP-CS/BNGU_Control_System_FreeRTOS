#ifndef APP_GIMBAL_COMMAND_MERGE_H
#define APP_GIMBAL_COMMAND_MERGE_H
#include "App/Vehicles/Sentry/Contracts/app_gimbal_data.h"
#include "App/Modules/can/app_can_data.h"

/* Latest pose-mode command wins; angular velocity is a persistent SI setpoint. */
static inline void app_gimbal_command_merge(app_gimbal_input_t *input,
                                            const app_can_ctrl_msg_t *message,
                                            uint8_t *radar_pending)
{
    const uint8_t pose = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT |
                         APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT |
                         APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT | APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
    if (message->updated_mask & pose)
    {
        input->updated_mask &= ~pose;
        input->can_valid_mask &= ~pose;
    }
    switch (message->type)
    {
        case APP_CAN_CMD_RADAR_SPEED:
            input->can.radar_speed = message->value.radar;
            *radar_pending = 1;
            break;
        case APP_CAN_CMD_SPEED_NO_SHOOT:
            input->can.speed_no_shoot.yaw_rate = message->value.angular_velocity.yaw_rate;
            input->can.speed_no_shoot.pitch_rate = message->value.angular_velocity.pitch_rate;
            break;
        case APP_CAN_CMD_SPEED_SHOOT:
            input->can.speed_shoot.yaw_rate = message->value.angular_velocity.yaw_rate;
            input->can.speed_shoot.pitch_rate = message->value.angular_velocity.pitch_rate;
            break;
        case APP_CAN_CMD_ANGLE_NO_SHOOT:
            input->updated_mask &= ~APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
            input->can.angle_no_shoot = message->value.angle;
            break;
        case APP_CAN_CMD_ANGLE_SHOOT:
            input->updated_mask &= ~APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT;
            input->can.angle_shoot = message->value.angle;
            break;
        case APP_CAN_CMD_SHOOT:
            input->can.shoot = message->value.shoot;
            break;
        default:
            return;
    }
    input->updated_mask |= message->updated_mask;
    input->can_valid_mask |= message->updated_mask;
    for (unsigned bit = 0; bit < 6; ++bit)
    {
        if (message->updated_mask & (1U << bit))
        {
            input->can_tick[bit] = message->tick;
        }
    }
}
#endif
