/** @file app_task_dbus_port.c
 * @brief Module-owned DMA instance and receive adapter; hardware is provided by Assembly.
 */
#include "app_task_dbus_port.h"
#include "app_task_dbus_internal.h"
#include "app_rtos.h"
static drv_dbus_port_t s_dbus;
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

bool app_task_dbus_port_start(UART_HandleTypeDef *uart)
{
    return drv_dbus_port_init(&s_dbus, uart, dbus_receive, NULL);
}
void app_task_dbus_port_stop(void)
{
    drv_dbus_port_stop(&s_dbus);
}
bool app_task_dbus_port_poll(void)
{
    return drv_dbus_port_poll(&s_dbus);
}
