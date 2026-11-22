/** Sentry gimbal-board launcher control task. */
#include "app_rtos.h"
#include "app_task_shoot_internal.h"
#include "app_tasks.h"
#include "app_rtos_config.h"
#include "app_vehicle_config.h"
#include "app_rtos_time.h"
#include "app_task_can.h"
#include "app_task_telemetry.h"
#include "app_sentry_shoot.h"
#include <string.h>

void Start_ctrl_shoot_Task(void *argument)
{
    (void)argument;
    uint32_t s_sequence = 0;
#if APP_BOARD_HAS_SHOOT
    static app_shoot_input_t input;
    static app_shoot_output_t output;
    app_tasks_wait_start(TASK_SHOOT);
    uint32_t next = app_rtos_start_tick + APP_RTOS_SHOOT_PERIOD;
    for (;;)
    {
        app_tasks_period_wait(TASK_SHOOT, &next, APP_RTOS_SHOOT_PERIOD);
        uint32_t started = app_rtos_cycles();
        /* 发射任务独占摩擦轮/拨弹轮 PID；不从 CAN 指令队列争抢消息。 */
        if (!app_rtos_shoot_read(&input.command))
        {
            app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        }
        if (!app_rtos_feedback_read(&input.feedback))
        {
            app_rtos_task_stats[TASK_SHOOT].snapshot_failures++;
        }
        input.tick = osKernelGetTickCount();
        app_sentry_launcher_ctrl_200hz(&input, &output);
        uint32_t tick = osKernelGetTickCount();
        app_rtos_shoot_publish(&output, tick);
        app_ctrl_can_msg_t m = {.tick = tick,
                                .sequence = ++s_sequence,
                                .source = TASK_SHOOT,
                                .type = APP_CAN_SHOOT_CURRENT};
        memcpy(m.value.current, output.current, sizeof(output.current));
        app_rtos_ctrl_send(&m);
        app_tasks_profile_end(TASK_SHOOT, started);
        next += APP_RTOS_SHOOT_PERIOD;
    }
#else
    app_tasks_idle_failed(TASK_SHOOT);
#endif
}
