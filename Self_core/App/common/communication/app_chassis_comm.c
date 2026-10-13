#include "app_chassis_comm.h"
#include <math.h>
#include <string.h>
_Static_assert(sizeof(app_chassis_speed_cmd_t)==8, "Chassis command must occupy eight bytes");
uint8_t app_chassis_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
    app_chassis_comm_rx_t *rx, uint8_t *mask)
{
    if (!bytes || !rx || !mask || len!=8) return 0;
    switch(id) {
    case APP_CHASSIS_CAN_ID_SPEED_CMD: memcpy(&rx->speed,bytes,8); *mask=1; break;
    case APP_CHASSIS_CAN_ID_ACKERMANN_CMD: {
        app_chassis_ackermann_cmd_t v; memcpy(&v,bytes,8);
        if (!isfinite(v.speed)||!isfinite(v.steer_angle)) return 0;
        rx->ackermann=v; *mask=2; break; }
    case APP_CHASSIS_CAN_ID_FOLLOW_CMD: memcpy(&rx->follow,bytes,8); *mask=4; break;
    default: return 0;
    }
    return 1;
}
