#include <math.h>
/** @file navigation_ctrl.c
 * @brief RTOS navigation adapter: source arbitration and synchronous wheel control.
 */
#include "navigation_ctrl.h"
#include "chassis.h"
#include "lib_math.h"
#include <string.h>
#include "app_data_age.h"
static uint8_t s_world_frame;

void navigation_ctrl_init(void)
{
    s_world_frame = 0;
    chassis_init();
}

void navigation_ctrl_step(const app_navigation_input_t *input, const app_dbus_snapshot_t *dbus,
                          const app_imu_snapshot_t *imu, uint32_t now_tick,
                          app_chassis_output_t *output)
{
    memset(output, 0, sizeof(*output));
    navigation_chassis_feedback(&input->chassis.feedback);
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    if (dbus->switch_s1 == 1U || dbus->switch_s1 == 3U)
    {
        if (dbus->switch_s2 == 3U)
        {
            s_world_frame = 0;
        }
        else if (dbus->switch_s2 == 1U)
        {
            s_world_frame = 1;
        }
    }
    if (!app_data_fresh(dbus->valid, dbus->tick, now_tick, APP_INPUT_DBUS_MAX_AGE_MS) ||
        !app_motor_feedback_fresh(&input->chassis.feedback, 0, 4, now_tick))
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
        vy = dbus->channel_norm[2] * -6.0f;
        wz = dbus->channel_norm[0] * -6.0f;
        if (s_world_frame)
        {
            if (!imu->valid || !imu->sampled || !imu->device_valid || !imu->filter_ready ||
                !app_data_fresh(true, imu->tick, now_tick, APP_INPUT_NAV_IMU_MAX_AGE_MS))
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
    else if (dbus->switch_s1 == 1U &&
             app_data_fresh(input->gamepad.valid, input->gamepad.tick, now_tick, 100))
    {
        vx = fabsf(input->gamepad.ly) <= .1f ? 0 : -6 * input->gamepad.ly;
        vy = fabsf(input->gamepad.lx) <= .1f ? 0 : -6 * input->gamepad.lx;
        wz = fabsf(input->gamepad.rx) <= .1f ? 0 : -6 * input->gamepad.rx;
        if (!s_world_frame)
        {
            (void)chassis_ctrl_body(vx, vy, wz);
        }
        else if (imu->valid && imu->sampled && imu->device_valid && imu->filter_ready &&
                 app_data_fresh(true, imu->tick, now_tick, APP_INPUT_NAV_IMU_MAX_AGE_MS))
        {
            (void)chassis_ctrl_world(vx, vy, wz, lib_rad_to_deg(imu->euler.yaw));
        }
        else
        {
            vx = vy = wz = 0;
            (void)chassis_stop();
        }
    }
    else if (dbus->switch_s1 == 2U && input->chassis.can.speed.power_pct > 0 &&
             input->chassis.can.speed.power_pct <= 1 &&
             app_data_fresh(input->chassis.command_valid, input->chassis.command_tick, now_tick,
                            APP_INPUT_CAN_MAX_AGE_MS))
    {
        s_world_frame = 0;
        vx = input->chassis.can.speed.vx;
        vy = input->chassis.can.speed.vy;
        wz = input->chassis.can.speed.vz;
        (void)chassis_ctrl_body(vx, vy, wz);
    }
    else
    {
        /* No valid selected input: stop all wheels. */
        (void)chassis_stop();
    }
    navigation_chassis_currents(output->current);
    output->target = (app_chassis_velocity_t){vx, vy, wz};
}
