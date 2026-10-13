#include "app_data.h"
#include "lib_math.h"
#include <math.h>
#include <float.h>
#include <string.h>
_Static_assert(sizeof(float)==4 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128, "CAN protocol requires float32");
_Static_assert(sizeof(app_gimbal_radar_cmd_t)==8, "Radar command must occupy eight bytes");
uint8_t app_gimbal_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
    app_gimbal_comm_rx_t *rx, uint8_t *mask)
{
    if (!bytes || !rx || !mask) return 0;
    if (id==APP_GIMBAL_CAN_ID_SHOOT_CMD) {
        if (len<4) return 0;
        rx->shoot.shoot_switch=bytes[0]; rx->shoot.retreat=bytes[1];
        *mask=APP_GIMBAL_COMM_UPDATE_SHOOT; return 1;
    }
    if (len!=8) return 0;
    float angles[2]; memcpy(angles,bytes,8);
    switch(id) {
    case APP_GIMBAL_CAN_ID_RADAR_SPEED: memcpy(&rx->radar_speed,bytes,8); *mask=APP_GIMBAL_COMM_UPDATE_RADAR_SPEED; break;
    case APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT:
    case APP_GIMBAL_CAN_ID_SPEED_SHOOT:
    case APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT:
    case APP_GIMBAL_CAN_ID_ANGLE_SHOOT:
        if (!isfinite(angles[0]) || !isfinite(angles[1])) return 0;
        if (id==APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT) { memcpy(&rx->speed_no_shoot,bytes,8); *mask=APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT; }
        if (id==APP_GIMBAL_CAN_ID_SPEED_SHOOT) { memcpy(&rx->speed_shoot,bytes,8); *mask=APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT; }
        if (id==APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT) { memcpy(&rx->angle_no_shoot,bytes,8); *mask=APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT; }
        if (id==APP_GIMBAL_CAN_ID_ANGLE_SHOOT) { memcpy(&rx->angle_shoot,bytes,8); *mask=APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT; } break;
    default: return 0;
    }
    return 1;
}
uint8_t app_gimbal_comm_dbus_rx(const app_dbus_snapshot_t *snapshot, app_gimbal_dbus_input_t *input)
{
    if (!snapshot || !snapshot->valid || !input) return 0;
    const drv_dbus_data_t *dbus=&snapshot->data;
    input->launcher_mode=dbus->rc.s1;
    input->chassis_vx_norm=lib_clamp(((float)dbus->rc.ch[0]-1024)/660,-1,1);
    input->chassis_vy_norm=lib_clamp(((float)dbus->rc.ch[1]-1024)/660,-1,1);
    input->yaw_rate_norm=lib_clamp(((float)dbus->rc.ch[2]-1024)/660,-1,1);
    input->pitch_rate_norm=lib_clamp(((float)dbus->rc.ch[3]-1024)/660,-1,1);
    input->chassis_omega_norm=lib_clamp(((float)dbus->rc.rolling_wheel-1024)/660,-1,1);
    input->source=(app_gimbal_input_source_t)dbus->rc.s2;
    return 1;
}
