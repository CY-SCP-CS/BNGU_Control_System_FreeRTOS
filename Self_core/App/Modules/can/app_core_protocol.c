#include "app_core_protocol.h"
#include "app_can_wire.h"
#include <string.h>

static int ref_index(uint32_t id)
{
    switch (id)
    {
        case 0x151:
            return 0;
        case 0x153:
            return 1;
        case 0x155:
            return 2;
        case 0x157:
            return 3;
        default:
            return -1;
    }
}

static unsigned ref_length(int i)
{
    static const unsigned n[4] = {2, 8, 8, 1};
    return n[i];
}

bool app_core_referee_decode(const app_can_frame_t *f, app_core_referee_t *state)
{
    if (!f || !state || f->bus != 1)
    {
        return false;
    }
    int i = ref_index(f->std_id);
    if (i < 0 || f->dlc != ref_length(i))
    {
        return false;
    }
    app_core_referee_t v = *state;
    switch (i)
    {
        case 0:
            v.heat_remaining = wire_i16(f->data);
            break;
        case 1:
            v.stage = f->data[0] & 15;
            v.hurt_type = f->data[0] >> 4;
            if (v.stage > 5 || (v.hurt_type != 0 && v.hurt_type != 1 && v.hurt_type != 5) ||
                (f->data[5] & 0xC0))
            {
                return false;
            }
            v.remaining_s = wire_u16(f->data + 1);
            v.warning_level = f->data[3];
            v.warning_robot = f->data[4];
            v.flags = f->data[5];
            v.recovery_pct = f->data[6];
            v.heat_ratio = f->data[7];
            break;
        case 2:
            v.hp = wire_u16(f->data);
            v.max_hp = wire_u16(f->data + 2);
            v.ally_1_hp = wire_u16(f->data + 4);
            v.ally_3_hp = wire_u16(f->data + 6);
            break;
        case 3:
            v.robot_id = f->data[0];
            break;
    }
    v.valid_mask |= (uint8_t)(1U << i);
    v.tick[i] = f->tick;
    *state = v;
    return true;
}

bool app_core_referee_encode(uint32_t id, const app_core_referee_t *v, app_can_frame_t *out)
{
    if (!v || !out)
    {
        return false;
    }
    int i = ref_index(id);
    if (i < 0 || !(v->valid_mask & (1U << i)))
    {
        return false;
    }
    app_can_frame_t f = {.bus = 1, .std_id = id, .dlc = (uint8_t)ref_length(i), .tick = v->tick[i]};
    switch (i)
    {
        case 0:
            wire_put_i16(f.data, v->heat_remaining);
            break;
        case 1:
            if (v->stage > 5 || (v->hurt_type != 0 && v->hurt_type != 1 && v->hurt_type != 5) ||
                (v->flags & 0xC0))
            {
                return false;
            }
            f.data[0] = v->stage | (v->hurt_type << 4);
            wire_put_u16(f.data + 1, v->remaining_s);
            f.data[3] = v->warning_level;
            f.data[4] = v->warning_robot;
            f.data[5] = v->flags;
            f.data[6] = v->recovery_pct;
            f.data[7] = v->heat_ratio;
            break;
        case 2:
            wire_put_u16(f.data, v->hp);
            wire_put_u16(f.data + 2, v->max_hp);
            wire_put_u16(f.data + 4, v->ally_1_hp);
            wire_put_u16(f.data + 6, v->ally_3_hp);
            break;
        case 3:
            f.data[0] = v->robot_id;
            break;
    }
    *out = f;
    return true;
}

bool app_core_imu_read_request(uint8_t bus, uint8_t rid, app_can_frame_t *out)
{
    if (!out || (bus != 1 && bus != 2) || rid < 1 || rid > 4)
    {
        return false;
    }
    *out = (app_can_frame_t){.bus = bus, .std_id = 0x031, .dlc = 4, .data = {0xCC, rid, 0, 0xDD}};
    return true;
}

bool app_core_imu_euler_decode(const app_can_frame_t *f, app_core_euler_t *out)
{
    if (!f || !out || f->std_id != 0x039 || f->dlc != 8 || f->data[0] != 3 || f->data[1] != 0)
    {
        return false;
    }
    const float k = 3.14159265358979323846f / 180.0f;
    *out = (app_core_euler_t){(wire_u16(f->data + 2) * (180.0f / 65535.0f) - 90) * k,
                              (wire_u16(f->data + 4) * (360.0f / 65535.0f) - 180) * k,
                              (wire_u16(f->data + 6) * (360.0f / 65535.0f) - 180) * k};
    return true;
}

bool app_core_quaternion_decode(const app_can_frame_t *f, float out[4])
{
    if (!f || !out || f->std_id != 0x233 || f->dlc != 8)
    {
        return false;
    }
    float q[4], norm = 0;
    for (unsigned i = 0; i < 4; i++)
    {
        int16_t n = wire_i16(f->data + 2 * i);
        if (n < -30000 || n > 30000)
        {
            return false;
        }
        q[i] = n / 30000.0f;
        norm += q[i] * q[i];
    }
    if (norm < 1e-12f)
    {
        return false;
    }
    norm = sqrtf(norm);
    for (unsigned i = 0; i < 4; i++)
    {
        out[i] = q[i] / norm;
    }
    return true;
}
