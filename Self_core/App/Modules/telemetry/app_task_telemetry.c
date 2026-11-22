/**
 * @file    app_task_telemetry.c
 * @brief   控制状态与波形的私有存储；三个控制发布者各自更新所属字段
 */
#include "app_rtos.h"
#include "app_task_telemetry_internal.h"
#include "app_rtos_objects.h"
#include <string.h>
#include "app_vehicle_config.h"
#include "lib_physical.h"

static osMutexId_t ctrl_state_mutex;
static app_ctrl_state_t s_ctrl;

bool app_rtos_ctrl_read(app_ctrl_state_t *value)
{
    return app_rtos_snapshot_read(ctrl_state_mutex, &s_ctrl, value, sizeof(*value));
}

/* 公共接口的参数、单位和失败策略以 app_task_telemetry.h 为准。 */
bool app_rtos_vofa_read(float value[9])
{
    if (!value || !ctrl_state_mutex || osMutexAcquire(ctrl_state_mutex, 0) != osOK)
    {
        return false;
    }
    /* 锁内只提取所需字段；不再保存一份完整控制状态作为读取工作区。 */
#if APP_BOARD_HAS_CHASSIS
    float channels[9] = {
        s_ctrl.chassis.target.vx_speed,
        s_ctrl.chassis.state.cur_speed.vx_speed,
        s_ctrl.chassis.target.vy_speed,
        s_ctrl.chassis.state.cur_speed.vy_speed,
        s_ctrl.chassis.target.vw_speed,
        s_ctrl.chassis.state.cur_speed.vw_speed,
        s_ctrl.chassis.state.cur_power,
        s_ctrl.chassis.state.power_scale,
        0.0f,
    };
#elif APP_BOARD_HAS_SHOOT
    float channels[9] = {
        s_ctrl.shoot.disc_nominal,      s_ctrl.shoot.disc_speed,
        s_ctrl.shoot.disc_integral,     s_ctrl.shoot.disc_effort_raw,
        -s_ctrl.shoot.friction_nominal, s_ctrl.shoot.friction_r_speed,
        s_ctrl.shoot.friction_nominal,  s_ctrl.shoot.friction_l_speed,
        s_ctrl.shoot.disc_encoder_raw,
    };
#else
    float channels[9] = {0};
#endif
    osMutexRelease(ctrl_state_mutex);
#if APP_BOARD_HAS_CHASSIS
    /* 保持原显示单位：前四个通道 m/s → mm/s，转换不占用共享锁。 */
    for (unsigned i = 0; i < 4; i++)
    {
        channels[i] = lib_m_to_mm(channels[i]);
    }
#endif
    memcpy(value, channels, sizeof(channels));
    return true;
}

void app_rtos_gimbal_publish(const app_gimbal_output_t *value, float roll, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.gimbal = *value;
        s_ctrl.gimbal_tick = tick;
        s_ctrl.imu_roll_rad = roll;
        osMutexRelease(ctrl_state_mutex);
    }
    else
    {
        app_rtos_count(COUNT_TELEMETRY_DROP);
    }
}

void app_rtos_shoot_publish(const app_shoot_output_t *value, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.shoot = *value;
        s_ctrl.shoot_tick = tick;
        osMutexRelease(ctrl_state_mutex);
    }
    else
    {
        app_rtos_count(COUNT_TELEMETRY_DROP);
    }
}

void app_rtos_chassis_publish(const app_chassis_output_t *value, uint32_t tick)
{
    if (osMutexAcquire(ctrl_state_mutex, 0) == osOK)
    {
        s_ctrl.chassis = *value;
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
