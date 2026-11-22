/** @file app_task_referee_port.c
 * @brief Module-owned DMA instance and receive adapter; hardware is provided by Assembly.
 */
#include "app_task_referee_port.h"
#include "app_task_referee_internal.h"
#include "app_rtos.h"
static drv_referee_port_t s_referee;
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

bool app_task_referee_port_start(UART_HandleTypeDef *uart)
{
    return drv_referee_port_init(&s_referee, uart, referee_receive, NULL);
}
void app_task_referee_port_stop(void)
{
    drv_referee_port_stop(&s_referee);
}
bool app_task_referee_port_poll(void)
{
    return drv_referee_port_poll(&s_referee);
}
