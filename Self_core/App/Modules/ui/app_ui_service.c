/** @file app_ui_service.c
 * @brief UI data assembly in referee task; disabled by default.
 */
#include "app_ui_service.h"
#include "app_ui_io.h"
#include "app_Armored_ui.h"
#include "ui_interface.h"
#include "app_ui_port.h"
#include "cmsis_os2.h"
#include <math.h>
#if APP_UI_MODE == 1
static uint8_t s_friction = 3; /* Unknown local shooter -> FAULT, never invent READY. */
static uint16_t s_heat, s_limit, s_ammo;
static float s_pitch_deg = NAN;
static uint8_t s_control;
#endif

void app_ui_service_init(void)
{
#if APP_UI_MODE != 0
    app_ui_port_init();
    app_ui_bind(osKernelGetTickCount, app_ui_port_send);
#if APP_UI_MODE == 2
    app_control_ui_init(); /* Source simulator fixture, explicitly opted in. */
#else
    app_Armored_ui_init(&s_friction, &s_heat, &s_limit, &s_ammo, &s_pitch_deg, &s_control);
#endif
#endif
}

void app_ui_service_poll(const app_ref_snapshot_t *referee)
{
#if APP_UI_MODE != 0
    app_ui_port_poll();
#if APP_UI_MODE == 1
    if (!referee || !referee->valid || !referee->robot_id ||
        (referee->valid_mask & ((1U << 6) | (1U << 7))) != ((1U << 6) | (1U << 7)))
    {
        return;
    }
    ui_self_id = referee->robot_id;
    s_heat = referee->heat_42_raw;
    s_limit = referee->heat_limit_raw;
    s_ammo = referee->projectile_42_raw;
#else
    (void)referee;
#endif
    app_Armored_ui_process();
#else
    (void)referee;
#endif
}
