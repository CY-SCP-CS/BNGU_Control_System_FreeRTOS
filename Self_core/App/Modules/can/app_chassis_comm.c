/** Core CAN1 chassis command codec, protocol 2026-11-16. */
#include "app_chassis_comm.h"
#include "app_can_wire.h"

uint8_t app_chassis_comm_decode(uint32_t id, const uint8_t *b, uint8_t len,
                                app_chassis_comm_rx_t *rx, uint8_t *mask)
{
    if (!b || !rx || !mask)
    {
        return 0;
    }
    if (id == 0x111 && len == 8)
    {
        int16_t power = wire_i16(b + 6);
        if (power < 0 || power > 10000)
        {
            return 0;
        }
        rx->speed = (app_chassis_speed_cmd_t){wire_i16(b) * .001f, wire_i16(b + 2) * .001f,
                                              wire_i16(b + 4) * .001f, power * .0001f};
        *mask = 1;
        return 1;
    }
    if (id == 0x113 && len == 4)
    {
        rx->ackermann =
            (app_chassis_ackermann_cmd_t){wire_i16(b) * .001f, wire_i16(b + 2) * .0001f};
        *mask = 2;
        return 1;
    }
    if (id == 0x115 && len == 8)
    {
        rx->follow = (app_chassis_follow_cmd_t){.vx = wire_i16(b) * .001f,
                                                .vy = wire_i16(b + 2) * .001f,
                                                .wz = wire_i16(b + 4) * .001f,
                                                .gimbal_angle = wire_i16(b + 6) * .0001f};
        *mask = 4;
        return 1;
    }
    return 0;
}
