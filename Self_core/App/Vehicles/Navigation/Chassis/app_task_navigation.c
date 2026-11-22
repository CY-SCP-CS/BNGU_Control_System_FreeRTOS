/** Independent navigation vehicle control: 1 ms RX service, 5 ms wheel control. */
#include "app_task_navigation.h"
#include "app_task_navigation_io.h"
#include "navigation_ctrl.h"
#include "app_tasks.h"
#include "app_rtos.h"
#include "app_rtos_config.h"
#include "app_rtos_time.h"
#include "app_task_can.h"
#include "app_task_dbus.h"
#include "app_task_imu.h"
#include "app_task_telemetry.h"
#include <string.h>

void Start_navigation_Task(void *argument)
{
    (void)argument;
    static app_navigation_input_t input;
    static app_chassis_output_t output;
    static app_dbus_snapshot_t dbus;
    static app_imu_snapshot_t imu;
    uint32_t sequence = 0, divider = 0;
    app_tasks_wait_start(TASK_CHAS);
    uint32_t next = app_rtos_start_tick + APP_RTOS_CHAS_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_CHAS, &next, APP_RTOS_CHAS_PERIOD);
        uint32_t started = app_rtos_cycles();
        app_navigation_gamepad_poll(&input.gamepad);
        if (!app_rtos_feedback_read(&input.chassis.feedback))
        {
            app_rtos_task_stats[TASK_CHAS].snapshot_failures++;
        }
        app_can_ctrl_msg_t msg;
        for (unsigned i = 0; i < APP_RTOS_CTRL_INPUT_BUDGET && app_rtos_can_ctrl_receive(&msg); i++)
        {
            if (msg.type == APP_CAN_CMD_CHASSIS_SPEED)
            {
                input.chassis.can.speed = msg.value.speed;
                input.chassis.command_tick = msg.tick;
                input.chassis.command_valid = 1;
            }
        }
        if (++divider >= (NAV_CONTROL_PERIOD_MS / NAV_RX_SERVICE_PERIOD_MS))
        {
            divider = 0;
            if (!app_rtos_dbus_read(&dbus))
            {
                app_rtos_task_stats[TASK_CHAS].snapshot_failures++;
            }
            if (!app_rtos_imu_read(&imu))
            {
                app_rtos_task_stats[TASK_CHAS].snapshot_failures++;
            }
            input.chassis.tick = osKernelGetTickCount();
            navigation_ctrl_step(&input, &dbus, &imu, input.chassis.tick, &output);
            app_rtos_chassis_publish(&output, input.chassis.tick);
            app_ctrl_can_msg_t m = {.tick = input.chassis.tick,
                                    .sequence = ++sequence,
                                    .source = TASK_CHAS,
                                    .type = APP_CAN_CHAS_CURRENT};
            memcpy(m.value.current, output.current, sizeof(output.current));
            app_rtos_ctrl_send(&m);
        }
        app_tasks_profile_end(TASK_CHAS, started);
        next += APP_RTOS_CHAS_PERIOD;
    }
}
