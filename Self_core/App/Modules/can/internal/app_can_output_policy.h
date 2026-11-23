#ifndef APP_CAN_OUTPUT_POLICY_H
#define APP_CAN_OUTPUT_POLICY_H
#include "App/Modules/can/app_can_data.h"
#include <string.h>
#include <stdbool.h>
#define APP_CAN_OUTPUT_MAX_AGE_MS 20U

static inline bool app_can_frame_prepare(app_can_frame_t *f, uint32_t now)
{
    uint32_t limit = (f->bus == 1 && f->std_id == 0x111) ? 100U : APP_CAN_OUTPUT_MAX_AGE_MS;
    if ((uint32_t)(now - f->tick) <= limit)
    {
        return true;
    }
    if (f->bus == 2 && (f->std_id == 0x200 || f->std_id == 0x1ff || f->std_id == 0x1fe))
    {
        memset(f->data, 0, 8);
    }
    else if (f->bus == 1 && f->std_id == 0x111)
    {
        memset(f->data, 0, 8);
        /* Explicit local timeout action: command zero velocity, no wire stop sentinel. */
    }
    else
    {
        return false;
    }
    return true;
}
#endif
