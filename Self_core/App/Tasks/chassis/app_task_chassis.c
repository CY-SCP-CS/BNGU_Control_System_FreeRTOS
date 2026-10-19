/**
 * @file    app_task_chassis.c
 * @brief   底盘输入整理、运动控制调用与电流提交
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_chassis.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_can.h"
#include "app_task_telemetry.h"
#include "app_sentry_chassis.h"
#include <string.h>

static uint32_t s_sequence;

/**
 * @brief  底盘控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅底盘板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   消费本板唯一的 CAN 指令队列，读取电机/功率计快照，计算并提交电流及 200 Hz 状态反馈。
 */
void Start_ctrl_chas_Task(void *argument)
{
    (void)argument;
    s_sequence = 0;
#if APP_BOARD_HAS_CHASSIS
    static app_chassis_input_t input;
    static app_chassis_output_t output;
    app_tasks_wait_start(TASK_CHAS);
    uint32_t next = app_rtos_start_tick + APP_RTOS_CHAS_PERIOD;
    uint32_t divider = 0;
    for (;;)
    {
        app_tasks_period_wait(TASK_CHAS, &next, APP_RTOS_CHAS_PERIOD);
        uint32_t started = app_rtos_cycles();
        if (!app_rtos_feedback_read(&input.feedback))
        {
            app_rtos_task_stats[TASK_CHAS].snapshot_failures++;
        }
        app_can_ctrl_msg_t received;
        /* 底盘板唯一指令消费者；0x113/0x115 只保存参数，不新增未实现模式。 */
        for (unsigned i = 0; i < APP_RTOS_CTRL_QUEUE_DEPTH && app_rtos_can_ctrl_receive(&received);
             i++)
        {
            if (received.type == APP_CAN_CMD_CHASSIS_SPEED)
            {
                input.can.speed = received.value.speed;
            }
            if (received.type == APP_CAN_CMD_ACKERMANN)
            {
                input.can.ackermann = received.value.ackermann;
            }
            if (received.type == APP_CAN_CMD_FOLLOW)
            {
                input.can.follow = received.value.follow;
            }
        }
        /* App 保留原固定 100 W 功率目标和功率计闭环，此处未加入裁判限功率。 */
        app_sentry_chassis_ctrl(&input, &output);
        uint32_t tick = osKernelGetTickCount();
        app_rtos_chassis_publish(&output, tick);
        app_ctrl_can_msg_t m = {.tick = tick,
                                .sequence = ++s_sequence,
                                .source = TASK_CHAS,
                                .type = APP_CAN_CHAS_CURRENT};
        memcpy(m.value.current, output.current, sizeof(output.current));
        app_rtos_ctrl_send(&m);
        if (++divider >= APP_RTOS_CHAS_FEEDBACK_DIVIDER)
        {
            divider = 0;
            m.type = APP_CAN_CHAS_FEEDBACK;
            m.value.chassis_state.power = output.state.cur_power;
            m.value.chassis_state.omega = output.omega;
            app_rtos_ctrl_send(&m);
        }
        app_tasks_profile_end(TASK_CHAS, started);
        next += APP_RTOS_CHAS_PERIOD;
    }
#else
    app_tasks_idle_failed(TASK_CHAS);
#endif
}
