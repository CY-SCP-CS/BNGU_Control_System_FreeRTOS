/** @file navigation_ctrl.c
 * @brief RTOS navigation adapter: source arbitration and synchronous wheel control.
 */
#include "navigation_ctrl.h"
#include "chassis.h"
#include "lib_math.h"
#include <string.h>
static uint8_t s_world_frame;

void navigation_ctrl_init(void)
{
    s_world_frame = 0;
    chassis_init();
}

void navigation_ctrl_step(const app_chassis_input_t *input, const app_dbus_snapshot_t *dbus,
                          const app_imu_snapshot_t *imu, uint32_t now_tick,
                          app_chassis_output_t *output)
{
    memset(output, 0, sizeof(*output));
    navigation_chassis_feedback(&input->feedback);
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    if (!dbus->valid)
    {
        (void)chassis_stop();
    }
    else if (dbus->switch_s1 == 3U)
    {
        if (dbus->switch_s2 == 3U)
        {
            s_world_frame = 0;
        }
        if (dbus->switch_s2 == 1U)
        {
            s_world_frame = 1;
        }
        vx = dbus->channel_norm[3] * 6.0f;
        vy = dbus->channel_norm[2] * 6.0f;
        wz = dbus->channel_norm[0] * 6.0f;
        if (s_world_frame)
        {
            if (!imu->valid || !imu->sampled || (uint32_t)(now_tick - imu->tick) > 20U)
            {
                vx = 0;
                vy = 0;
                wz = 0;
                (void)chassis_stop();
            }
            else
            {
                (void)chassis_ctrl_world(vx, vy, wz, lib_rad_to_deg(imu->euler.yaw));
            }
        }
        else
        {
            (void)chassis_ctrl_body(vx, vy, wz);
        }
    }
    else if (dbus->switch_s1 == 2U)
    {
        s_world_frame = 0;
        vx = input->can.speed.vx;
        vy = input->can.speed.vy;
        wz = input->can.speed.vz;
        (void)chassis_ctrl_body(vx, vy, wz);
    }
    else
    {
        /* Bare-metal game-controller mode has no implementation. */
        (void)chassis_stop();
    }
    navigation_chassis_currents(output->current);
    output->target = (app_sentry_chassis_speed_t){vx, vy, wz};
}
