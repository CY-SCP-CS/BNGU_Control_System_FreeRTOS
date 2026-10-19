/**
 * @file    app_task_gimbal.c
 * @brief   云台输入整理、算法调用与结果提交
 * @note    入口名、周期、业务算法及原通信对象名保持；通信对象句柄仅所属模块可访问。
 */
#include "app_rtos.h"
#include "app_task_gimbal.h"
#include "app_tasks.h"
#include "app_rtos_objects.h"
#include "app_rtos_config.h"
#include "app_board_config.h"
#include "app_rtos_time.h"
#include "app_task_can.h"
#include "app_task_dbus.h"
#include "app_task_imu.h"
#include "app_task_shoot.h"
#include "app_task_telemetry.h"
#include "app_sentry_gimbal.h"
#include <string.h>

static uint32_t s_sequence;
/* CAN 指令队列只有本板主控制任务消费，发射任务读取云台发布的已裁决快照。 */
#if APP_BOARD_HAS_GIMBAL
static app_gimbal_input_t s_gim_input;   //云台任务私有输入；快照读失败时保留上周期副本
static app_gimbal_output_t s_gim_output; //云台任务私有输出

/**
 * @brief  合并本周期云台 CAN 指令并生成更新掩码
 * @param  radar_pending 有效输出状态指针；收到 0x120 时置 1，供 200 Hz 转发阶段消费
 * @note   本板唯一的 can_to_ctrl_queue 消费者；每次最多取 16 条，同 ID 保留最后一条。
 * @note   每周期先清更新掩码，增量只应用一次，实际顺序由 App 算法保持裸机规则。
 */
static void gimbal_drain(uint8_t *radar_pending)
{
    app_can_ctrl_msg_t m;
    s_gim_input.updated_mask = 0;
    for (unsigned i = 0; i < APP_RTOS_CTRL_QUEUE_DEPTH && app_rtos_can_ctrl_receive(&m); i++)
    {
        switch (m.type)
        {
            case APP_CAN_CMD_RADAR_SPEED:
                s_gim_input.can.radar_speed = m.value.radar;
                *radar_pending = 1;
                break;
            case APP_CAN_CMD_SPEED_NO_SHOOT:
                s_gim_input.can.speed_no_shoot = m.value.delta;
                break;
            case APP_CAN_CMD_SPEED_SHOOT:
                s_gim_input.can.speed_shoot = m.value.delta;
                break;
            case APP_CAN_CMD_ANGLE_NO_SHOOT:
                s_gim_input.can.angle_no_shoot = m.value.angle;
                break;
            case APP_CAN_CMD_ANGLE_SHOOT:
                s_gim_input.can.angle_shoot = m.value.angle;
                break;
            case APP_CAN_CMD_SHOOT:
                s_gim_input.can.shoot = m.value.shoot;
                break;
            default:
                continue;
        }
        s_gim_input.updated_mask |= m.updated_mask;
    }
}
#endif
/**
 * @brief  云台控制任务入口
 * @param  argument     CMSIS 任务入口参数，当前创建时传 NULL
 * @note   仅云台板创建；SYS_READY 后独立使用 osDelayUntil()，周期 1 tick / 1 ms。
 * @note   读取 DBUS/IMU/CAN 快照，裁决控制源并执行双 yaw/pitch 控制；发布发射命令和 200 Hz 板间反馈。
 */
void Start_ctrl_gim_Task(void *argument)
{
    (void)argument;
    s_sequence = 0;
#if APP_BOARD_HAS_GIMBAL
    app_tasks_wait_start(TASK_GIM);
    uint32_t next = app_rtos_start_tick + APP_RTOS_GIM_PERIOD;
    uint32_t divider = 0;
    uint8_t radar_pending = 0;
    for (;;)
    {
        app_tasks_period_wait(TASK_GIM, &next, APP_RTOS_GIM_PERIOD);
        uint32_t started = app_rtos_cycles();
        /* 快照读锁超时为 0，失败沿用上一份完整输入并计数，不等待本周期 IMU DMA。 */
        s_gim_input.tick = osKernelGetTickCount();
        if (!app_rtos_dbus_read(&s_gim_input.dbus))
        {
            app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        }
        if (!app_rtos_imu_read(&s_gim_input.imu))
        {
            app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        }
        if (!app_rtos_feedback_read(&s_gim_input.feedback))
        {
            app_rtos_task_stats[TASK_GIM].snapshot_failures++;
        }
        uint32_t age = s_gim_input.tick - s_gim_input.imu.tick;
        if (age > app_rtos_task_stats[TASK_GIM].max_imu_age_ms)
        {
            app_rtos_task_stats[TASK_GIM].max_imu_age_ms = age;
        }
        /* 汇总本周期 CAN 更新掩码后进入 App；控制源、急停和限位仍由业务算法裁决。 */
        gimbal_drain(&radar_pending);
        app_sentry_gimbal_ctrl_1khz(&s_gim_input, &s_gim_output);
        s_gim_output.shoot.sequence = ++s_sequence;
        /* 发射只使用已裁决命令；本任务优先级高于发射，同 tick 到期时先发布。 */
        app_rtos_shoot_write(&s_gim_output.shoot);
        app_rtos_gimbal_publish(&s_gim_output, s_gim_input.imu.euler.roll, s_gim_input.tick);
        app_ctrl_can_msg_t m = {.tick = s_gim_input.tick,
                                .sequence = s_sequence,
                                .source = TASK_GIM,
                                .type = APP_CAN_GIM_CURRENT};
        memcpy(m.value.current, s_gim_output.current, sizeof(s_gim_output.current));
        app_rtos_ctrl_send(&m);
        /* 控制每 1 ms 运行，5 轮分频为 200 Hz，保持原板间命令/姿态反馈节奏。 */
        if (++divider == APP_RTOS_GIM_FEEDBACK_DIVIDER)
        {
            divider = 0;
            if (s_gim_output.shoot.source != APP_GIMBAL_INPUT_CAN || radar_pending)
            {
                m.type = APP_CAN_CHASSIS_CMD;
                m.value.chassis = s_gim_output.chassis;
                app_rtos_ctrl_send(&m);
            }
            radar_pending = 0;
            m.type = APP_CAN_GIM_FEEDBACK;
            m.value.gimbal.yaw_rate = s_gim_output.yaw_rate;
            m.value.gimbal.pitch_rate = s_gim_output.pitch_rate;
            m.value.gimbal.yaw = s_gim_output.yaw;
            m.value.gimbal.pitch = s_gim_output.pitch;
            m.value.gimbal.quat = s_gim_input.imu.quat;
            app_rtos_ctrl_send(&m);
        }
        app_tasks_profile_end(TASK_GIM, started);
        next += APP_RTOS_GIM_PERIOD;
    }
#else
    app_tasks_idle_failed(TASK_GIM);
#endif
}
