/** Core CAN1 gimbal command codec; physical SI values, never per-frame increments. */
#include "app_gimbal_comm.h"
#include "app_can_wire.h"

uint8_t app_gimbal_comm_decode(uint32_t id, const uint8_t *b, uint8_t len, app_gimbal_comm_rx_t *rx,
                               uint8_t *mask)
{
    if (!b || !rx || !mask)
    {
        return 0;
    }
    if (id == 0x129 && len == 2)
    {
        if ((b[0] != 0 && b[0] != 255) || (b[1] != 0 && b[1] != 170 && b[1] != 255))
        {
            return 0;
        }
        rx->shoot = (app_gimbal_shoot_cmd_t){b[0], b[1]};
        *mask = APP_GIMBAL_COMM_UPDATE_SHOOT;
        return 1;
    }
    if (id == 0x120 && len == 8)
    {
        int16_t power = wire_i16(b + 6);
        if (power < 0 || power > 10000)
        {
            return 0;
        }
        rx->radar_speed = (app_gimbal_radar_cmd_t){wire_i16(b) * .001f, wire_i16(b + 2) * .001f,
                                                   wire_i16(b + 4) * .001f, power * .0001f};
        *mask = 1;
        return 1;
    }
    if (len != 4)
    {
        return 0;
    }
    if (id == 0x121 || id == 0x125)
    {
        app_gimbal_speed_cmd_t v = {wire_i16(b) * .001f, wire_i16(b + 2) * .001f};
        if (id == 0x121)
        {
            rx->speed_no_shoot = v;
            *mask = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT;
        }
        else
        {
            rx->speed_shoot = v;
            *mask = APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT;
        }
        return 1;
    }
    if (id == 0x123 || id == 0x127)
    {
        app_gimbal_angle_cmd_t v = {wire_i16(b) * .0001f, wire_i16(b + 2) * .0001f};
        if (id == 0x123)
        {
            rx->angle_no_shoot = v;
            *mask = APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT;
        }
        else
        {
            rx->angle_shoot = v;
            *mask = APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
        }
        return 1;
    }
    return 0;
}
