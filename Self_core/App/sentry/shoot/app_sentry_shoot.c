/**
 * @file    app_sentry_shoot.c
 * @brief   哨兵发射机构算法：按已裁决模式计算摩擦轮和拨弹轮电流。
 * @note    发射任务独占 PID；调度、快照获取和 CAN 提交由 App/Tasks 完成。
 */
#include "app_control_config.h"
#include "app_motor_role.h"
#include "app_sentry_shoot.h"
#include "app_sentry_common.h"
#include "lib_pid.h"
#include "lib_math.h"
#include <string.h>

/* 索引沿用云台 CAN 反馈表：0~2 为云台电机，3/4/5 为左摩擦轮/拨弹轮/右摩擦轮。 */

/* 以下缓存、模式和 PID 都是发射任务私有状态；跨任务结果通过输出快照发布。 */
static struct
{
    drv_motor_data_t feedback[GIMBAL_MOTOR_COUNT];
    int16_t current[GIMBAL_MOTOR_COUNT];
} s_motor_state;

static struct
{
    struct
    {
        uint8_t fire;
    } data;
} s_command_state;

static lib_pid_t s_pid_disc, s_pid_fric_l, s_pid_fric_r;

/**
 * @brief  清零三路发射电流并复位拨弹轮和摩擦轮 PID。
 * @note   用于未就绪、急停或未知模式，和 FIRE_OFF 的摩擦轮零速闭环行为不同。
 */
static void launcher_reset(void)
{
    s_motor_state.current[MOTOR_FRIC_L] = 0;
    s_motor_state.current[MOTOR_FRIC_R] = 0;
    s_motor_state.current[MOTOR_DISC] = 0;
    lib_pid_reset(&s_pid_disc);
    lib_pid_reset(&s_pid_fric_l);
    lib_pid_reset(&s_pid_fric_r);
}

/**
 * @brief  按关闭、连续发射、仅摩擦轮或退弹模式计算电机电流。
 * @note   左右摩擦轮目标方向相反；退弹只反转拨弹轮，摩擦轮仍按原目标转动。
 * @note   FIRE_OFF 将摩擦轮目标设为零并继续速度闭环，拨弹轮电流清零；未知模式直接复位。
 */
static void launch_control(void)
{
    switch (s_command_state.data.fire)
    {
        case APP_SENTRY_FIRE_OFF:
            s_motor_state.current[MOTOR_DISC] = 0;
            lib_pid_reset(&s_pid_disc);
            s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
                &s_pid_fric_l, 0.0f,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
                &s_pid_fric_r, 0.0f,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed),
                APP_CONTROL_SHOOT_DT_S);

            break;
        default:
            launcher_reset();

            break;

        case APP_SENTRY_FIRE_ON:
            s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
                &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
                &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
                &s_pid_disc, SENTRY_DISC_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
                APP_CONTROL_SHOOT_DT_S);
            break;

        case APP_SENTRY_FIRE_FIR:
            s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
                &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
                &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_DISC] = 0;
            lib_pid_reset(&s_pid_disc);
            break;

        case APP_SENTRY_FIRE_REVERSE:
            s_motor_state.current[MOTOR_FRIC_L] = (int16_t)lib_pid_calc(
                &s_pid_fric_l, SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_FRIC_R] = (int16_t)lib_pid_calc(
                &s_pid_fric_r, -SENTRY_FRICTION_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed),
                APP_CONTROL_SHOOT_DT_S);
            s_motor_state.current[MOTOR_DISC] = (int16_t)lib_pid_calc(
                &s_pid_disc, -SENTRY_DISC_TAR_SPEED,
                lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed),
                APP_CONTROL_SHOOT_DT_S);
            break;
    }
}

/**
 * @brief  清空发射反馈缓存并初始化拨弹轮和两只摩擦轮速度 PID。
 * @note   由初始化任务调用；后续 PID 和内部电流状态只由 ctrl_shoot_Task 修改。
 */
void app_sentry_shoot_init(void)
{
    memset(&s_motor_state, 0, sizeof(s_motor_state));
    lib_pid_init(&s_pid_disc, 100, 0, 0, 0, 0, 0, 0, DRV_MOTOR_M2006_CURRENT_MIN,
                 DRV_MOTOR_M2006_CURRENT_MAX, 2000);
    lib_pid_init(&s_pid_fric_l, 15, 1, 0, 0, 0, 0, 0, DRV_MOTOR_M3508_CURRENT_MIN,
                 DRV_MOTOR_M3508_CURRENT_MAX, 1000);
    lib_pid_init(&s_pid_fric_r, 15, 1, 0, 0, 0, 0, 0, DRV_MOTOR_M3508_CURRENT_MIN,
                 DRV_MOTOR_M3508_CURRENT_MAX, 1000);
}

/**
 * @brief  完成一次 5 ms 发射控制并输出三路电流和具名遥测值。
 * @param  input        已裁决发射命令及电机反馈快照，不可为 NULL。
 * @param  output       输出电流顺序为左摩擦轮、拨弹轮、右摩擦轮；不可为 NULL。
 * @note   由 ctrl_shoot_Task 周期调用；步长来自 APP_CONTROL_SHOOT_DT_S，默认 5 ms。
 * @note   ready 未置位或 emergency_stop 有效时清零电流并复位 PID；其他情况按 fire 模式控制。
 * @note   本函数不再直接读取 DBUS，也不发送 CAN；模式已由云台控制任务统一裁决。
 */
void app_sentry_launcher_ctrl_200hz(const app_shoot_input_t *input, app_shoot_output_t *output)
{
    memcpy(s_motor_state.feedback, input->feedback.motor, sizeof(s_motor_state.feedback));
    s_command_state.data.fire = input->command.fire;
    if (!input->command.ready || input->command.emergency_stop)
    {
        launcher_reset();
    }
    else
    {
        launch_control();
    }
    memcpy(output->current, &s_motor_state.current[MOTOR_FRIC_L], sizeof(output->current));
    output->disc_nominal = SENTRY_DISC_TAR_SPEED;
    output->disc_speed = lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_DISC].speed);
    output->friction_nominal = SENTRY_FRICTION_TAR_SPEED;
    output->friction_l_speed = lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_L].speed);
    output->friction_r_speed = lib_rpm_to_rad_s((float)s_motor_state.feedback[MOTOR_FRIC_R].speed);
    output->friction_r_current = s_motor_state.feedback[MOTOR_FRIC_R].current;
}
