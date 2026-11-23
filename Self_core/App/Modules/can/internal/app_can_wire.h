#ifndef APP_CAN_WIRE_H
#define APP_CAN_WIRE_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "lib_physical.h"

static inline uint16_t wire_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int16_t wire_i16(const uint8_t *p)
{
    uint16_t u = wire_u16(p);
    return (int16_t)(u < 32768U ? (int32_t)u : (int32_t)u - 65536);
}

static inline void wire_put_u16(uint8_t *p, uint16_t u)
{
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}

static inline void wire_put_i16(uint8_t *p, int16_t v)
{
    wire_put_u16(p, (uint16_t)v);
}

static inline bool wire_quantize(uint8_t *p, float value, float step)
{
    if (!isfinite(value))
    {
        return false;
    }
    wire_put_i16(p, lib_quantize_i16(value, step));
    return true;
}

/* Kept as convenient test/value layouts; codecs never memcpy structs to wire. */
typedef struct
{
    int16_t vx, vy, vz, power_pct;
} wire_chassis_speed_cmd_t;

typedef struct
{
    int16_t speed, steer_angle;
} wire_chassis_ackermann_cmd_t;

typedef struct
{
    int16_t vx, vy, wz, gimbal_angle;
} wire_chassis_follow_cmd_t;

typedef wire_chassis_speed_cmd_t wire_gimbal_radar_cmd_t;
#endif
