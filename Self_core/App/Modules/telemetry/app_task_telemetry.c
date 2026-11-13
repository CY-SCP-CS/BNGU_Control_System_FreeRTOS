/**
 * @file    app_task_telemetry.c
 * @brief   控制状态与波形的私有存储；三个控制发布者各自更新所属字段
 */
#include "app_rtos.h"
#include "app_task_telemetry_internal.h"
#include "app_rtos_objects.h"
#include <string.h>
#include "app_board_config.h"
#include "lib_physical.h"

static osMutexId_t ctrl_state_mutex;
static app_ctrl_state_t s_ctrl;

/**
 * @brief  读取完整控制状态快照
 * @param  v            输出控制状态，锁不可用时保留旧副本
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   任务上下文调用，ctrl_state_mutex 保护；超时为 0。
 */
bool app_rtos_ctrl_read(app_ctrl_state_t *v)
{
    return app_rtos_snapshot_read(ctrl_state_mutex, &s_ctrl, v, sizeof(*v));
}

/**
 * @brief  读取控制快照中的 VOFA 九通道
 * @param  v            至少容纳 9 个 float 的输出数组，失败保留旧值
 * @return true=复制成功，false=参数/句柄无效或锁暂不可用
 * @note   使用 ctrl_state_mutex，任务上下文获取锁超时为 0。
 */
bool app_rtos_vofa_read(float v[9])
{
    static app_ctrl_state_t state;
    if (!v || !app_rtos_ctrl_read(&state))
    {
        return false;
    }
#if APP_BOARD_HAS_CHASSIS
    v[0] = lib_m_to_mm(state.chassis.target.vx_speed);
    v[1] = lib_m_to_mm(state.chassis.state.cur_speed.vx_speed);
    v[2] = lib_m_to_mm(state.chassis.target.vy_speed);
    v[3] = lib_m_to_mm(state.chassis.state.cur_speed.vy_speed);
    v[4] = state.chassis.target.vw_speed;
    v[5] = state.chassis.state.cur_speed.vw_speed;
    v[6] = state.chassis.state.cur_power;
    v[7] = state.chassis.state.power_scale;
    v[8] = 0.0f;
#else
    v[0] = state.shoot.disc_nominal;
    v[1] = state.shoot.disc_speed;
    v[2] = state.shoot.disc_integral;
    v[3] = state.shoot.disc_effort_raw;
    v[4] = -state.shoot.friction_nominal;
    v[5] = state.shoot.friction_r_speed;
    v[6] = state.shoot.friction_nominal;
    v[7] = state.shoot.friction_l_speed;
    v[8] = state.shoot.disc_encoder_raw;
#endif
    return true;
}

/**
 * @brief  发布云台控制输出与 IMU roll 诊断值
 * @param  v            有效的云台输出指针
 * @param  roll         IMU roll，rad；仅保存诊断，不占用发射 VOFA 通道
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由云台控制任务调用；持有 ctrl_state_mutex 只更新云台字段，保留发射字段。
 */
void app_rtos_gimbal_publish(const app_gimbal_output_t *v, float roll, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.gimbal = *v;
        s_ctrl.gimbal_tick = tick;
        s_ctrl.imu_roll_rad = roll;
        osMutexRelease(ctrl_state_mutex);
    }
    else
    {
        app_rtos_count(COUNT_TELEMETRY_DROP);
    }
}

/**
 * @brief  发布发射输出及原发射 VOFA 七通道
 * @param  v            有效的发射输出指针，包含电流及具名遥测值
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由发射任务调用；持有 ctrl_state_mutex 更新发射字段，保留云台发布的通道 7。
 */
void app_rtos_shoot_publish(const app_shoot_output_t *v, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.shoot = *v;
        s_ctrl.shoot_tick = tick;
        osMutexRelease(ctrl_state_mutex);
    }
    else
    {
        app_rtos_count(COUNT_TELEMETRY_DROP);
    }
}

/**
 * @brief  发布底盘控制输出及 VOFA 九通道
 * @param  v            有效的底盘输出指针
 * @param  tick         本轮控制时间，单位内核 tick
 * @note   由底盘控制任务调用；通道依次为 x/y/omega 目标与实际值、功率、缩放系数。
 */
void app_rtos_chassis_publish(const app_chassis_output_t *v, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.chassis = *v;
        s_ctrl.chassis_tick = tick;
        osMutexRelease(ctrl_state_mutex);
    }
    else
    {
        app_rtos_count(COUNT_TELEMETRY_DROP);
    }
}

/**
 * @brief  创建本模块全部通信对象并复位数据
 * @return true=全部创建成功，false=创建失败，由装配层统一回滚。
 * @note   只在调度器启动前调用一次；不支持运行中重新初始化。
 */
bool app_task_telemetry_init(void)
{
    memset(&s_ctrl, 0, sizeof(s_ctrl));
    return app_rtos_object_mutex(&ctrl_state_mutex, "ctrl_state_mutex");
}

/** @brief 启动失败时删除已创建对象；句柄为 NULL 时安全跳过。 */
void app_task_telemetry_deinit(void)
{
    app_rtos_object_mutex_delete(&ctrl_state_mutex);
}
