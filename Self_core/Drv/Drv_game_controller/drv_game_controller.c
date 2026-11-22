/** Pure JOY text parser, independent of UART and float scanf linker options. */
#include "drv_game_controller.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
uint8_t drv_game_controller_decode(const uint8_t *buffer, drv_game_controller_buttons_t *data)
{
    if (!buffer || !data || strncmp((const char *)buffer, "JOY,", 4))
    {
        return 0;
    }
    const char *p = (const char *)buffer + 4;
    float v[4];
    for (unsigned i = 0; i < 4; i++)
    {
        char *end;
        v[i] = strtof(p, &end);
        if (end == p || !isfinite(v[i]) || fabsf(v[i]) > 1)
        {
            return 0;
        }
        if (i < 3)
        {
            if (*end != ',')
            {
                return 0;
            }
            p = end + 1;
        }
        else
        {
            while (*end == ' ' || *end == '\r' || *end == '\n')
            {
                end++;
            }
            if (*end)
            {
                return 0;
            }
        }
    }
    data->lx = v[0];
    data->ly = v[1];
    data->rx = v[2];
    data->ry = v[3];
    return 1;
}
