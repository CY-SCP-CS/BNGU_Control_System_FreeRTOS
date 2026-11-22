/** @file app_task_imu_port.c
 * @brief IMU sampling and recovery adapter; static buffers belong to this module.
 */
#include "app_task_imu_port.h"
#include "app_task_imu_internal.h"
#include "app_rtos.h"
#include "app_task_monitor.h"
#include "drv_mag_port.h"
static drv_imu_port_t s_imu;
static drv_imu_port_config_t s_config;
void app_task_imu_port_configure(const drv_imu_port_config_t *config)
{
    s_config = *config;
}
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

bool app_task_imu_port_prepare(drv_imu_t *imu)
{
    bool ok = drv_imu_port_init(&s_imu, imu, &s_config, imu_receive, NULL, delay_ms, NULL);
    if (ok)
    {
        /* Missing external magnetometer is diagnostic, not an initialization failure. */
        (void)drv_mag_port_init();
    }
    return ok;
}

void app_task_imu_port_poll(void)
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

void app_task_imu_port_trigger(uint32_t tick, uint32_t sequence, uint32_t cycles)
{
    (void)drv_imu_port_sample_trigger(&s_imu, tick, sequence, cycles);
}

uint32_t app_task_imu_port_errors(void)
{
    return drv_imu_port_get_error_count(&s_imu);
}

uint32_t app_task_imu_port_busy(void)
{
    return drv_imu_port_get_busy_count(&s_imu);
}
