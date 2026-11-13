/** @file app_devices.c
 * @brief 哨兵硬件实例装配；回调转换事件，缓冲和配置静态存活。
 */
#include "app_devices.h"
#include "app_board_config.h"
#include "app_rtos.h"
#include "app_task_dbus_internal.h"
#include "app_task_referee_internal.h"
#include "app_task_imu_internal.h"
#include "drv_dbus_port.h"
#include "drv_referee_port.h"
#include "drv_imu_port.h"
#include "drv_mag_port.h"
#include "app_task_monitor.h"
static drv_dbus_port_t s_dbus;
static drv_referee_port_t s_referee;
static drv_imu_port_t s_imu;

#if APP_BOARD_HAS_DBUS
static void dbus_receive(void *context, drv_io_event_t event, const uint8_t *bytes, uint16_t length)
{
    (void)context;
    (void)length;
    if (event == DRV_IO_DATA)
    {
        app_rtos_dbus_receive_isr(bytes);
    }
    else
    {
        app_rtos_count(COUNT_UART_ERROR);
        app_rtos_notify_task(TASK_DBUS, APP_RTOS_TF_DATA_READY);
    }
}

#endif
#if APP_BOARD_HAS_REFEREE
static void referee_receive(void *context, drv_io_event_t event, const uint8_t *bytes,
                            uint16_t length)
{
    (void)context;
    if (event == DRV_IO_DATA)
    {
        app_rtos_ref_push_isr(bytes, length);
    }
    else
    {
        app_rtos_ref_error_isr();
    }
}

#endif
static void imu_receive(void *context, drv_io_event_t event, const drv_imu_dma_sample_t *sample)
{
    (void)context;
    if (event == DRV_IO_DATA)
    {
        app_rtos_imu_receive(sample);
    }
    else
    {
        app_rtos_notify_task(TASK_IMU, APP_RTOS_TF_DATA_READY);
    }
}

static void delay_ms(void *context, uint32_t ms)
{
    (void)context;
    app_rtos_heartbeat(TASK_INIT);
    if (osDelay(ms) != osOK)
    {
        app_rtos_fatal_code = 3;
    }
}

bool app_devices_start(void)
{
#if APP_BOARD_HAS_DBUS
    if (!drv_dbus_port_init(&s_dbus, &huart3, dbus_receive, NULL))
    {
        app_devices_stop();
        return false;
    }
#endif
#if APP_BOARD_HAS_REFEREE
    if (!drv_referee_port_init(&s_referee, &huart6, referee_receive, NULL))
    {
        app_devices_stop();
        return false;
    }
#endif
    return true;
}

void app_devices_stop(void)
{
    drv_dbus_port_stop(&s_dbus);
    drv_referee_port_stop(&s_referee);
}

bool app_devices_dbus_poll(void)
{
    return drv_dbus_port_poll(&s_dbus);
}

bool app_devices_referee_poll(void)
{
    return drv_referee_port_poll(&s_referee);
}

bool app_devices_imu_prepare(drv_imu_t *imu)
{
    const drv_imu_port_config_t config = {&hspi1,     GPIOA,   GPIO_PIN_4,   GPIOB,
                                          GPIO_PIN_0, &htim10, TIM_CHANNEL_1};
    bool ok = drv_imu_port_init(&s_imu, imu, &config, imu_receive, NULL, delay_ms, NULL);
    if (ok)
    {
        /* Missing external magnetometer is diagnostic, not an initialization failure. */
        (void)drv_mag_port_init();
    }
    return ok;
}

void app_devices_imu_poll(void)
{
    drv_imu_port_poll(&s_imu);
    drv_mag_port_process();
    drv_mag_snapshot_t mag;
    if (drv_mag_port_read_snapshot(&mag))
    {
        /* HAL and kernel ticks have different epochs; convert sample age, not timestamp. */
        uint32_t age_ms = HAL_GetTick() - mag.last_update_ms;
        app_rtos_module_update(MODULE_MAG, mag.valid && mag.online,
                               osKernelGetTickCount() - age_ms);
        app_rtos_module_diagnostics(MODULE_MAG, mag.error_count, mag.recovery_count);
    }
}

void app_devices_imu_trigger(uint32_t tick, uint32_t sequence, uint32_t cycles)
{
    (void)drv_imu_port_sample_trigger(&s_imu, tick, sequence, cycles);
}

uint32_t app_devices_imu_errors(void)
{
    return drv_imu_port_get_error_count(&s_imu);
}

uint32_t app_devices_imu_busy(void)
{
    return drv_imu_port_get_busy_count(&s_imu);
}

bool app_devices_mag_read(float field_tesla[3], uint32_t *sample_tick)
{
    drv_mag_snapshot_t snapshot;
    if (!field_tesla || !sample_tick || !drv_mag_port_read_snapshot(&snapshot) || !snapshot.valid)
    {
        return false;
    }
    field_tesla[0] = snapshot.sample.x_uT * 1.0e-6f;
    field_tesla[1] = snapshot.sample.y_uT * 1.0e-6f;
    field_tesla[2] = snapshot.sample.z_uT * 1.0e-6f;
    *sample_tick = snapshot.last_update_ms;
    return true;
}
