/**
 * @file    app_chassis_comm.c
 * @brief   底盘板间指令解包：把 CAN 数据转换成控制任务私有状态。
 */
#include "app_chassis_comm.h"
#include <math.h>
#include "app_can_wire.h"
#include "lib_physical.h"
#include <string.h>

_Static_assert(sizeof(wire_chassis_speed_cmd_t) == 8, "Chassis command must occupy eight bytes");

/**
 * @brief  解包一帧底盘 CAN1 指令，更新对应字段及本帧更新掩码。
 * @param  id           CAN 标准 ID，支持 0x111、0x113、0x115。
 * @param  bytes        只读载荷，函数内部复制数据，不保存该地址。
 * @param  len          载荷长度，必须为 8 字节。
 * @param  rx           输入/输出：调用者持有的完整协议状态，仅覆盖本 ID 对应字段。
 * @param  mask         输出：0x111/0x113/0x115 分别对应位 0/1/2；成功时赋值。
 * @return 1=识别并接受该帧，0=参数、长度、ID 或浮点数据无效；失败时不要使用输出掩码。
 * @note   纯协议函数，不操作队列、锁或 HAL。当前控制算法使用 0x111，另两类指令只解析存储。
 * @note   多字节字段沿用 STM32 小端协议；多个帧的掩码应由调用者按位或汇总。
 */
uint8_t app_chassis_comm_decode(uint32_t id, const uint8_t *bytes, uint8_t len,
                                app_chassis_comm_rx_t *rx, uint8_t *mask)
{
    if (!bytes || !rx || !mask || len != 8)
    {
        return 0;
    }
    switch (id)
    {
        case APP_CHASSIS_CAN_ID_SPEED_CMD:
        {
            wire_chassis_speed_cmd_t speed;
            memcpy(&speed, bytes, 8);
            rx->speed = (app_chassis_speed_cmd_t){lib_mm_to_m(speed.vx), lib_mm_to_m(speed.vy),
                                                  speed.vz * 0.001f, speed.power_pct * 0.0001f};
            *mask = 1;
            break;
        }
        case APP_CHASSIS_CAN_ID_ACKERMANN_CMD:
        {
            wire_chassis_ackermann_cmd_t v;
            memcpy(&v, bytes, 8);
            if (!isfinite(v.speed) || !isfinite(v.steer_angle))
            {
                return 0;
            }
            rx->ackermann = (app_chassis_ackermann_cmd_t){lib_mm_to_m(v.speed), v.steer_angle};
            *mask = 2;
            break;
        }
        case APP_CHASSIS_CAN_ID_FOLLOW_CMD:
        {
            wire_chassis_follow_cmd_t follow;
            memcpy(&follow, bytes, 8);
            rx->follow = (app_chassis_follow_cmd_t){lib_mm_to_m(follow.vx), lib_mm_to_m(follow.vy),
                                                    follow.gimbal_angle * 0.001f, follow.custom};
            *mask = 4;
            break;
        }
        default:
            return 0;
    }
    return 1;
}
