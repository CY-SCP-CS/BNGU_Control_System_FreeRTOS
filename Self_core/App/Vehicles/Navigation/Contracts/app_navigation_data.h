#ifndef APP_NAVIGATION_DATA_H
#define APP_NAVIGATION_DATA_H
#include "app_chassis_data.h"

typedef struct
{
    float lx, ly, rx, ry;
    uint32_t tick;
    uint8_t valid;
} app_gamepad_snapshot_t;

typedef struct
{
    app_chassis_input_t chassis;
    app_gamepad_snapshot_t gamepad;
} app_navigation_input_t;
#endif
