/**
 * @file    app_gimbal_comm.c
 * @brief   云台 CAN 板间指令解包。
 */
#include "app_gimbal_comm.h"
#include <math.h>
#include "app_can_wire.h"
#include "lib_physical.h"
#include <float.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "CAN protocol requires float32");
_Static_assert(sizeof(wire_gimbal_radar_cmd_t) == 8, "Radar command must occupy eight bytes");

/**
 * @brief  按 CAN ID 解包云台指令，返回本帧对应的更新位。
 * @param  id           CAN 标准 ID，支持 0x120/0x121/0x123/0x125/0x127/0x129。
 * @param  bytes        只读载荷，数据复制到 rx，函数不保留输入指针。
 * @param  len          除 0x129 至少 4 字节外，其他已支持指令必须为 8 字节。
 * @param  rx           输入/输出：调用者的协议缓存，仅覆盖当前 ID 对应字段。
 * @param  mask         输出：APP_GIMBAL_COMM_UPDATE_* 中对应的一位；成功时赋值。
 * @return 1=解包成功，0=空参数、未知 ID、长度不符或角度含非有限浮点数。
 * @note   任务上下文的纯协议函数；0x121/0x125 的实际含义为角度增量，单位 rad。
 * @note   每帧的更新位由调用者汇总；同 ID 的最新数据覆盖旧值，增量只应在更新周期使用一次。
 */
uint8_t app_gimbal_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
                               app_gimbal_comm_rx_t *rx, uint8_t *mask)
{
    if (!bytes || !rx || !mask)
    {
        return 0;
    }
    if (id == APP_GIMBAL_CAN_ID_SHOOT_CMD)
    {
        if (len < 4)
        {
            return 0;
        }
        rx->shoot.shoot_switch = bytes[0];
        rx->shoot.retreat = bytes[1];
        *mask = APP_GIMBAL_COMM_UPDATE_SHOOT;
        return 1;
    }
    if (len != 8)
    {
        return 0;
    }
    float angles[2];
    memcpy(angles, bytes, 8);
    switch (id)
    {
        case APP_GIMBAL_CAN_ID_RADAR_SPEED:
        {
            wire_gimbal_radar_cmd_t radar;
            memcpy(&radar, bytes, 8);
            rx->radar_speed =
                (app_gimbal_radar_cmd_t){lib_mm_to_m(radar.vx), lib_mm_to_m(radar.vy),
                                         radar.vz * 0.001f, radar.power_pct * 0.0001f};
            *mask = APP_GIMBAL_COMM_UPDATE_RADAR_SPEED;
            break;
        }
        case APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT:
        case APP_GIMBAL_CAN_ID_SPEED_SHOOT:
        case APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT:
        case APP_GIMBAL_CAN_ID_ANGLE_SHOOT:
            if (!isfinite(angles[0]) || !isfinite(angles[1]))
            {
                return 0;
            }
            if (id == APP_GIMBAL_CAN_ID_SPEED_NO_SHOOT)
            {
                memcpy(&rx->speed_no_shoot, bytes, 8);
                *mask = APP_GIMBAL_COMM_UPDATE_SPEED_NO_SHOOT;
            }
            if (id == APP_GIMBAL_CAN_ID_SPEED_SHOOT)
            {
                memcpy(&rx->speed_shoot, bytes, 8);
                *mask = APP_GIMBAL_COMM_UPDATE_SPEED_SHOOT;
            }
            if (id == APP_GIMBAL_CAN_ID_ANGLE_NO_SHOOT)
            {
                memcpy(&rx->angle_no_shoot, bytes, 8);
                *mask = APP_GIMBAL_COMM_UPDATE_ANGLE_NO_SHOOT;
            }
            if (id == APP_GIMBAL_CAN_ID_ANGLE_SHOOT)
            {
                memcpy(&rx->angle_shoot, bytes, 8);
                *mask = APP_GIMBAL_COMM_UPDATE_ANGLE_SHOOT;
            }
            break;
        default:
            return 0;
    }
    return 1;
}
