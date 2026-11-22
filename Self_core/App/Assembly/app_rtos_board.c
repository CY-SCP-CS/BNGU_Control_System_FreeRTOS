/**
 * @file    app_rtos_board.c
 * @brief   STM32F407 哨兵的板级装配与中断路由
 * @note    此处知道硬件和服务两端；Bsp/Drv 只上调注册的回调，不反向包含 Os 头。
 */

#include "app_rtos_board.h"
#include "app_vehicle_config.h"
#include "app_rtos.h"
#include "app_rtos_time.h"
#include "app_task_can_port.h"
#include "app_task_log_port.h"
#include "app_task_imu_internal.h"

#include "app_task_imu_port.h"
#if CURRENT_ROBOT == ROBOT_NAVIGATION && CURRENT_BOARD == BOARD_CHASSIS
#include "navigation_ctrl.h"
#elif CURRENT_ROBOT == ROBOT_ARMORED && CURRENT_BOARD == BOARD_CHASSIS
#include "app_Armored_mecanum.h"
#elif APP_BOARD_HAS_GIMBAL
#include "app_sentry_gimbal.h"
#include "app_sentry_shoot.h"
#elif CURRENT_ROBOT == ROBOT_SENTRY
#include "app_sentry_chassis.h"
#endif
#include "bsp_tim.h"
#include "drv_led.h"
#include "drv_buzzer.h"
static void app_devices_stop(void);
#include "app_vehicle_config.h"
#include "app_task_dbus_port.h"
#include "app_task_referee_port.h"
#include "app_task_imu_port.h"

static void app_devices_configure(void)
{
    const drv_imu_port_config_t config = {
        .spi = &hspi1,
        .acc_gpio = GPIOA,
        .acc_pin = GPIO_PIN_4,
        .gyro_gpio = GPIOB,
        .gyro_pin = GPIO_PIN_0,
        .heater = &htim10,
        .heater_channel = TIM_CHANNEL_1,
    };
    app_task_imu_port_configure(&config);
}

static bool app_devices_start(void)
{
#if APP_BOARD_HAS_DBUS
    if (!app_task_dbus_port_start(&huart3))
    {
        app_devices_stop();
        return false;
    }
#endif
#if APP_BOARD_HAS_REFEREE
    if (!app_task_referee_port_start(&huart6))
    {
        app_devices_stop();
        return false;
    }
#endif
    return true;
}

static void app_devices_stop(void)
{
    app_task_dbus_port_stop();
    app_task_referee_port_stop();
}

#if APP_BOARD_HAS_IMU
static uint32_t s_sample_seq;
static bool s_imu_timer_registered;

/**
 * @brief  记录 TIM14 采样时间并尝试启动一份 IMU DMA 采样。
 * @return 无返回值；忙或启动失败由设备端口累计诊断计数。
 * @note   定时器中断上下文，不阻塞，不执行姿态或控制计算。
 */
static void imu_tick(void)
{
    app_task_imu_port_trigger(osKernelGetTickCount(), ++s_sample_seq, app_rtos_cycles());
}
#endif

/**
 * @brief  初始化当前板型使用的算法、PID 和滤波状态。
 * @return 无返回值；同步初始化算法状态，不执行外设启动。
 * @note   仅 sys_init_Task 在发布 READY 前通过板级初始化调用一次。
 */
static void app_init(void)
{
#if CURRENT_ROBOT == ROBOT_NAVIGATION && CURRENT_BOARD == BOARD_CHASSIS
    navigation_ctrl_init();
#elif CURRENT_ROBOT == ROBOT_ARMORED && CURRENT_BOARD == BOARD_CHASSIS
    /* Separate vehicle: wheel algorithms initialized; no complete control task defined. */
    mecnum_init();
#elif APP_BOARD_HAS_GIMBAL
    app_sentry_gimbal_init();
    app_sentry_shoot_init();
#elif CURRENT_ROBOT == ROBOT_SENTRY
    app_sentry_chassis_init();
#endif
}

void app_rtos_board_init(void)
{
    app_init();
    app_devices_configure();
    app_rtos_port_init();
    app_task_log_port_init();
    drv_led_port_init();
    drv_buzzer_port_init();
#if APP_BOARD_HAS_CAN
    app_task_can_port_bind();
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
