/**
 * @file    app_rtos_board.c
 * @brief   STM32F407 哨兵的板级装配与中断路由
 * @note    此处知道硬件和服务两端；Bsp/Drv 只上调注册的回调，不反向包含 Os 头。
 */

#include "app_rtos_board.h"
#include "app_board_config.h"
#include "app_rtos.h"
#include "app_rtos_time.h"
#include "app_task_can_port.h"
#include "app_task_log_port.h"
#include "app_task_can_internal.h"
#include "app_task_dbus_internal.h"
#include "app_task_referee_internal.h"
#include "app_task_imu_internal.h"
#include "app_init.h"
#include "app_devices.h"
#include "bsp_tim.h"
#include "drv_led.h"
#include "drv_buzzer.h"
#if APP_BOARD_HAS_IMU
static uint32_t s_sample_seq;
static bool s_imu_timer_registered;
#endif
#if APP_BOARD_HAS_IMU
/** @brief TIM14 唯一运行采样触发，只记录时间并尝试启动有界 DMA。 */
static void imu_tick(void)
{
    app_devices_imu_trigger(osKernelGetTickCount(), ++s_sample_seq, app_rtos_cycles());
}

#endif
void app_rtos_board_init(void)
{
    app_init();
    app_rtos_port_init();
    app_task_log_port_init();
    drv_led_port_init();
    drv_buzzer_port_init();
#if APP_BOARD_HAS_CAN
    app_task_can_port_bind();
#endif
#if APP_BOARD_HAS_DBUS
#endif
#if APP_BOARD_HAS_REFEREE
#endif
#if APP_BOARD_HAS_IMU
    s_sample_seq = 0;
    s_imu_timer_registered = bsp_tim_reg_callback(&htim14, imu_tick) == HAL_OK;
#endif
}

bool app_rtos_ports_start(void)
{
#if APP_BOARD_HAS_IMU
    /* 必须先接好采样中断入口，注册失败由初始化任务发布失败状态。 */
    if (!s_imu_timer_registered)
    {
        return false;
    }
#endif
#if APP_BOARD_HAS_CAN
    if (!app_task_can_port_start())
    {
        return false;
    }
#endif
#if APP_BOARD_HAS_DBUS || APP_BOARD_HAS_REFEREE
    if (!app_devices_start())
    {
        return false;
    }
#endif

    return true;
}

bool app_rtos_board_calibrate(void)
{
#if APP_BOARD_HAS_IMU
    return app_rtos_imu_prepare();
#else
    return true;
#endif
}

bool app_rtos_sampling_start(void)
{
#if APP_BOARD_HAS_IMU
    __HAL_TIM_SET_COUNTER(&htim14, 0);
    __HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
    return HAL_TIM_Base_Start_IT(&htim14) == HAL_OK;
#else
    return true;
#endif
}
